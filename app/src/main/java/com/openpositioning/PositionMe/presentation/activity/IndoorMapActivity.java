package com.openpositioning.PositionMe.presentation.activity;

import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.PolygonOptions;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.utils.IndoorSelectionStore;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

public class IndoorMapActivity extends AppCompatActivity implements OnMapReadyCallback {

    private static final String TAG = "IndoorMapActivity";

    private GoogleMap mMap;

    private JSONObject venue;

    private TextView tvFloor;
    private Button btnDown, btnUp;

    // Floors discovered from map_shapes
    private final List<String> floors = new ArrayList<>();
    private int floorIndex = 0;

    // floor -> list of polygons (each polygon is list of LatLng)
    private final HashMap<String, List<List<LatLng>>> floorToPolygons = new HashMap<>();

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_indoor_map);

        tvFloor = findViewById(R.id.tvFloor);
        btnDown = findViewById(R.id.btnFloorDown);
        btnUp = findViewById(R.id.btnFloorUp);

        // Load selected venue from SharedPreferences (Step 4A)
        String venueJson = IndoorSelectionStore.getSelectedVenueJson(this);
        if (venueJson == null || venueJson.trim().isEmpty()) {
            Toast.makeText(this, "No venue selected. Go back and tap a building.", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        try {
            venue = new JSONObject(venueJson);
        } catch (Exception e) {
            Log.e(TAG, "Failed to parse stored venue JSON", e);
            Toast.makeText(this, "Stored venue JSON is invalid.", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        String name = venue.optString("name", "Indoor Venue");
        setTitle("Indoor: " + name);

        // Parse map_shapes now (even before map is ready)
        parseMapShapesIntoFloors(venue);

        btnDown.setOnClickListener(v -> {
            if (floors.isEmpty()) return;
            if (floorIndex > 0) {
                floorIndex--;
                updateFloorLabel();
                redrawCurrentFloor();
            }
        });

        btnUp.setOnClickListener(v -> {
            if (floors.isEmpty()) return;
            if (floorIndex < floors.size() - 1) {
                floorIndex++;
                updateFloorLabel();
                redrawCurrentFloor();
            }
        });

        updateFloorLabel();

        SupportMapFragment mapFragment =
                (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.indoorMap);
        if (mapFragment != null) {
            mapFragment.getMapAsync(this);
        } else {
            Toast.makeText(this, "Map fragment missing in layout.", Toast.LENGTH_LONG).show();
            finish();
        }
    }

    @Override
    public void onMapReady(@NonNull GoogleMap googleMap) {
        mMap = googleMap;

        // Always draw building outline as context
        List<LatLng> outline = extractOutlineLatLngs(venue);
        if (outline != null && outline.size() >= 3) {
            mMap.addPolygon(new PolygonOptions().addAll(outline));
        }

        // If we have real floor shapes, draw the current floor
        if (!floors.isEmpty()) {
            redrawCurrentFloor();
        } else {
            // Fallback: zoom to outline only
            if (outline != null && !outline.isEmpty()) {
                zoomToPoints(outline, 80);
            }
            Toast.makeText(this, "No map_shapes floors returned for this venue.", Toast.LENGTH_LONG).show();
        }
    }

    private void redrawCurrentFloor() {
        if (mMap == null) return;
        if (floors.isEmpty()) return;

        String floorKey = floors.get(floorIndex);

        // Clear and redraw: easiest + clean for coursework
        mMap.clear();

        // Re-draw outline
        List<LatLng> outline = extractOutlineLatLngs(venue);
        if (outline != null && outline.size() >= 3) {
            mMap.addPolygon(new PolygonOptions().addAll(outline));
        }

        // Draw floor polygons
        List<List<LatLng>> polys = floorToPolygons.get(floorKey);
        if (polys == null || polys.isEmpty()) {
            Toast.makeText(this, "No shapes for floor: " + floorKey, Toast.LENGTH_SHORT).show();
            if (outline != null && !outline.isEmpty()) zoomToPoints(outline, 80);
            return;
        }

        List<LatLng> allPts = new ArrayList<>();

        for (List<LatLng> poly : polys) {
            if (poly.size() >= 3) {
                mMap.addPolygon(new PolygonOptions().addAll(poly));
                allPts.addAll(poly);
            }
        }

        if (!allPts.isEmpty()) {
            zoomToPoints(allPts, 120);
        } else if (outline != null && !outline.isEmpty()) {
            zoomToPoints(outline, 80);
        }

        Toast.makeText(this, "Showing floor: " + floorKey, Toast.LENGTH_SHORT).show();
        Log.d(TAG, "Rendered floor=" + floorKey + " polygons=" + polys.size());
    }

    private void updateFloorLabel() {
        if (tvFloor == null) return;
        if (floors.isEmpty()) {
            tvFloor.setText("Floor: (none)");
        } else {
            tvFloor.setText("Floor: " + floors.get(floorIndex));
        }
    }

    /**
     * Parse venue.map_shapes (stringified GeoJSON FeatureCollection).
     * Expected: { "type":"FeatureCollection", "features":[ ... ] }
     * Each feature should have a properties object containing some floor indicator.
     */
    private void parseMapShapesIntoFloors(@NonNull JSONObject venueObj) {
        floors.clear();
        floorToPolygons.clear();

        try {
            String mapShapesStr = venueObj.optString("map_shapes", null);
            if (mapShapesStr == null || mapShapesStr.trim().isEmpty()) {
                Log.w(TAG, "map_shapes missing/empty in venue JSON");
                return;
            }

            if (!mapShapesStr.trim().startsWith("{")) {
                Log.w(TAG, "map_shapes is not JSON object text");
                return;
            }

            JSONObject fc = new JSONObject(mapShapesStr);
            JSONArray features = fc.optJSONArray("features");
            if (features == null || features.length() == 0) {
                Log.w(TAG, "map_shapes has no features");
                return;
            }

            for (int i = 0; i < features.length(); i++) {
                JSONObject feature = features.optJSONObject(i);
                if (feature == null) continue;

                JSONObject props = feature.optJSONObject("properties");
                JSONObject geom = feature.optJSONObject("geometry");
                if (geom == null) continue;

                // Floor key: try common fields (we don’t guess new APIs; we just adapt)
                String floorKey = extractFloorKey(props);
                if (floorKey == null) floorKey = "unknown";

                String type = geom.optString("type", "");
                JSONArray coords = geom.optJSONArray("coordinates");

                List<List<LatLng>> polyList = new ArrayList<>();

                if ("Polygon".equalsIgnoreCase(type)) {
                    List<LatLng> poly = parsePolygon(coords);
                    if (poly != null) polyList.add(poly);
                } else if ("MultiPolygon".equalsIgnoreCase(type)) {
                    polyList.addAll(parseMultiPolygon(coords));
                } else {
                    // ignore LineString/Point/etc for now
                    continue;
                }

                if (polyList.isEmpty()) continue;

                List<List<LatLng>> bucket = floorToPolygons.get(floorKey);
                if (bucket == null) {
                    bucket = new ArrayList<>();
                    floorToPolygons.put(floorKey, bucket);
                }
                bucket.addAll(polyList);
            }

            floors.addAll(floorToPolygons.keySet());

            // Sort floors in a human-ish order: LG, G, 1, 2, 3... else alphabetical fallback
            Collections.sort(floors, new Comparator<String>() {
                @Override
                public int compare(String a, String b) {
                    return floorSortValue(a) - floorSortValue(b);
                }
            });

            // Default to G if it exists
            int gIndex = floors.indexOf("G");
            if (gIndex >= 0) floorIndex = gIndex;
            else floorIndex = 0;

            Log.d(TAG, "Parsed map_shapes floors=" + floors + " buckets=" + floorToPolygons.size());

        } catch (Exception e) {
            Log.e(TAG, "Failed to parse map_shapes", e);
        }
    }

    @Nullable
    private String extractFloorKey(@Nullable JSONObject props) {
        if (props == null) return null;

        // Common candidates (depends on dataset)
        String[] keys = new String[]{"floor", "level", "storey", "story", "z", "layer"};
        for (String k : keys) {
            if (props.has(k)) {
                String v = props.optString(k, null);
                if (v != null && !v.trim().isEmpty() && !"null".equalsIgnoreCase(v.trim())) {
                    return v.trim();
                }
                // If numeric
                if (!props.isNull(k)) {
                    double n = props.optDouble(k, Double.NaN);
                    if (!Double.isNaN(n)) return String.valueOf((int) n);
                }
            }
        }
        return null;
    }

    private int floorSortValue(@NonNull String f) {
        // Put common ones first
        if (f.equalsIgnoreCase("LG")) return 0;
        if (f.equalsIgnoreCase("G"))  return 1;

        // numeric?
        try {
            int n = Integer.parseInt(f);
            return 10 + n;
        } catch (Exception ignored) { }

        // Unknown floors after
        return 1000 + f.toUpperCase().charAt(0);
    }

    // ===== Outline parsing (your venue.outline is a GeoJSON string) =====

    @Nullable
    private List<LatLng> extractOutlineLatLngs(@NonNull JSONObject venueObj) {
        try {
            String outlineStr = venueObj.optString("outline", null);
            if (outlineStr == null || !outlineStr.trim().startsWith("{")) return null;

            JSONObject outlineGeo = new JSONObject(outlineStr);
            JSONArray features = outlineGeo.optJSONArray("features");
            if (features == null || features.length() == 0) return null;

            JSONObject f0 = features.optJSONObject(0);
            if (f0 == null) return null;

            JSONObject geom = f0.optJSONObject("geometry");
            if (geom == null) return null;

            String type = geom.optString("type", "");
            JSONArray coords = geom.optJSONArray("coordinates");

            if ("MultiPolygon".equalsIgnoreCase(type)) {
                List<List<LatLng>> polys = parseMultiPolygon(coords);
                return polys.isEmpty() ? null : polys.get(0);
            } else if ("Polygon".equalsIgnoreCase(type)) {
                return parsePolygon(coords);
            }
        } catch (Exception e) {
            Log.e(TAG, "Outline parse failed", e);
        }
        return null;
    }

    // ===== GeoJSON parsing helpers =====

    @Nullable
    private List<LatLng> parsePolygon(@Nullable JSONArray coordinates) {
        if (coordinates == null) return null;

        // Polygon: coordinates[0] is outer ring
        JSONArray ring = coordinates.optJSONArray(0);
        if (ring == null) return null;

        List<LatLng> pts = new ArrayList<>();
        for (int i = 0; i < ring.length(); i++) {
            JSONArray pair = ring.optJSONArray(i);
            if (pair == null || pair.length() < 2) continue;

            double lon = pair.optDouble(0, Double.NaN);
            double lat = pair.optDouble(1, Double.NaN);
            if (!Double.isNaN(lat) && !Double.isNaN(lon)) pts.add(new LatLng(lat, lon));
        }
        return pts.size() >= 3 ? pts : null;
    }

    @NonNull
    private List<List<LatLng>> parseMultiPolygon(@Nullable JSONArray coordinates) {
        List<List<LatLng>> out = new ArrayList<>();
        if (coordinates == null) return out;

        // MultiPolygon: [ polygon1, polygon2, ... ]
        for (int p = 0; p < coordinates.length(); p++) {
            JSONArray poly = coordinates.optJSONArray(p);
            if (poly == null) continue;

            // poly[0] = outer ring
            JSONArray ring = poly.optJSONArray(0);
            if (ring == null) continue;

            List<LatLng> pts = new ArrayList<>();
            for (int i = 0; i < ring.length(); i++) {
                JSONArray pair = ring.optJSONArray(i);
                if (pair == null || pair.length() < 2) continue;

                double lon = pair.optDouble(0, Double.NaN);
                double lat = pair.optDouble(1, Double.NaN);
                if (!Double.isNaN(lat) && !Double.isNaN(lon)) pts.add(new LatLng(lat, lon));
            }

            if (pts.size() >= 3) out.add(pts);
        }
        return out;
    }

    private void zoomToPoints(@NonNull List<LatLng> pts, int paddingPx) {
        if (mMap == null || pts.isEmpty()) return;

        LatLngBounds.Builder b = new LatLngBounds.Builder();
        for (LatLng p : pts) b.include(p);

        LatLngBounds bounds = b.build();
        mMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds, paddingPx));
    }
}
