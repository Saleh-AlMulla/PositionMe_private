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
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.utils.IndoorSelectionStore;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;

public class IndoorMapActivity extends AppCompatActivity implements OnMapReadyCallback {

    private static final String TAG = "IndoorMapActivity";

    private GoogleMap mMap;
    private JSONObject venue;

    private TextView tvFloor;
    private Button down, up;

    // Floors discovered from map_shapes (ordered)
    private final List<String> floors = new ArrayList<>();
    private int floorIndex = 0;

    // We keep drawn polygons so we can clear/redraw on floor change
    private final List<Polygon> drawnFloorPolys = new ArrayList<>();

    // Outline points for zooming
    private List<LatLng> venueOutline;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_indoor_map);

        tvFloor = findViewById(R.id.tvFloor);
        down = findViewById(R.id.btnFloorDown);
        up = findViewById(R.id.btnFloorUp);

        // Load last selected venue
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

        Log.d(TAG, "Venue keys in IndoorMapActivity: " + venue.names());
        Object ms = venue.opt("map_shapes");
        Log.d(TAG, "map_shapes type in IndoorMapActivity: " +
                (ms == null ? "null" : ms.getClass().getSimpleName()));

        setTitle("Indoor: " + venue.optString("name", "Venue"));

        SupportMapFragment mapFragment =
                (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.indoorMap);
        if (mapFragment != null) {
            mapFragment.getMapAsync(this);
        } else {
            Toast.makeText(this, "Map fragment missing in layout.", Toast.LENGTH_LONG).show();
            finish();
        }

        down.setOnClickListener(v -> {
            if (floors.isEmpty()) return;
            if (floorIndex > 0) {
                floorIndex--;
                updateFloorLabel();
                redrawFloor();
            }
        });

        up.setOnClickListener(v -> {
            if (floors.isEmpty()) return;
            if (floorIndex < floors.size() - 1) {
                floorIndex++;
                updateFloorLabel();
                redrawFloor();
            }
        });
    }

    @Override
    public void onMapReady(@NonNull GoogleMap googleMap) {
        mMap = googleMap;

        // 1) Draw outline (always)
        venueOutline = extractOutlineLatLngs(venue);
        if (venueOutline != null && venueOutline.size() >= 3) {
            mMap.addPolygon(new PolygonOptions().addAll(venueOutline));
            LatLng center = computeCentroid(venueOutline);
            mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(center, 19f));

            // optional: slight upward shift because your floor selector bar is at the top
            mMap.animateCamera(CameraUpdateFactory.scrollBy(0f, -120f));
        }

        // 2) Parse map_shapes into floor->list of polygons
        Map<String, List<List<LatLng>>> floorToPolys = parseMapShapesToFloorPolygons(venue);

        if (floorToPolys.isEmpty()) {
            Toast.makeText(this,
                    "No drawable floor shapes found in map_shapes for this venue.",
                    Toast.LENGTH_LONG).show();
            // Still OK: outline proves selection + activity works (4A/4B wiring).
            floors.clear();
            floors.add("N/A");
            floorIndex = 0;
            updateFloorLabel();
            return;
        }

        floors.clear();
        floors.addAll(floorToPolys.keySet()); // LinkedHashMap preserves insertion order
        floorIndex = Math.min(floorIndex, floors.size() - 1);
        updateFloorLabel();

        // Store parsed shapes for redraw
        this.cachedFloorToPolys = floorToPolys;

        // 3) Draw initial floor
        redrawFloor();
    }

    // Cache of parsed floor polygons
    private Map<String, List<List<LatLng>>> cachedFloorToPolys = new LinkedHashMap<>();

    private void redrawFloor() {
        if (mMap == null) return;

        // Clear old floor polys
        for (Polygon p : drawnFloorPolys) p.remove();
        drawnFloorPolys.clear();

        if (floors.isEmpty()) return;

        String floorKey = floors.get(floorIndex);
        List<List<LatLng>> polys = cachedFloorToPolys.get(floorKey);
        if (polys == null || polys.isEmpty()) {
            Toast.makeText(this, "No shapes for floor " + floorKey, Toast.LENGTH_SHORT).show();
            return;
        }

        int drawn = 0;
        for (List<LatLng> pts : polys) {
            if (pts != null && pts.size() >= 3) {
                Polygon p = mMap.addPolygon(new PolygonOptions().addAll(pts));
                drawnFloorPolys.add(p);
                drawn++;
            }
        }

        Toast.makeText(this, "Floor " + floorKey + " drawn: " + drawn + " shape(s)", Toast.LENGTH_SHORT).show();
    }

    private void updateFloorLabel() {
        if (tvFloor == null) return;
        if (floors.isEmpty()) tvFloor.setText("Floor: N/A");
        else tvFloor.setText("Floor: " + floors.get(floorIndex));
    }

    // =========================
    // Outline parsing (your API)
    // =========================

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

            return parseGeometryToLatLngs(geom);
        } catch (Exception e) {
            Log.e(TAG, "Outline parse failed", e);
            return null;
        }
    }

    // =========================================
    // map_shapes parsing (robust “Option A”)
    // =========================================

    @NonNull
    private Map<String, List<List<LatLng>>> parseMapShapesToFloorPolygons(@NonNull JSONObject venueObj) {
        Map<String, List<List<LatLng>>> out = new LinkedHashMap<>();

        Object mapShapes = venueObj.opt("map_shapes");
        if (mapShapes == null) {
            Log.d(TAG, "map_shapes missing");
            return out;
        }

        try {
            // map_shapes might be a String containing JSON
            if (mapShapes instanceof String) {
                String s = ((String) mapShapes).trim();
                if (s.startsWith("{")) mapShapes = new JSONObject(s);
                else if (s.startsWith("[")) mapShapes = new JSONArray(s);
            }

            // Handle FeatureCollection in object form
            if (mapShapes instanceof JSONObject) {
                JSONObject obj = (JSONObject) mapShapes;

                // Common: { "type":"FeatureCollection", "features":[...]}
                if ("FeatureCollection".equalsIgnoreCase(obj.optString("type"))) {
                    JSONArray features = obj.optJSONArray("features");
                    consumeFeaturesIntoFloors(features, out);
                    return out;
                }

                // Some APIs: { "floors": { "G": {FeatureCollection}, "1": {...} } }
                // If object keys look like floors, just try each key as a floor bucket.
                JSONArray names = obj.names();
                if (names != null) {
                    for (int i = 0; i < names.length(); i++) {
                        String key = names.optString(i);
                        Object v = obj.opt(key);
                        JSONObject fc = null;
                        if (v instanceof String) {
                            String s = ((String) v).trim();
                            if (s.startsWith("{")) fc = new JSONObject(s);
                        } else if (v instanceof JSONObject) {
                            fc = (JSONObject) v;
                        }
                        if (fc != null && "FeatureCollection".equalsIgnoreCase(fc.optString("type"))) {
                            JSONArray features = fc.optJSONArray("features");
                            consumeFeaturesIntoFloorsWithFixedFloor(features, out, key);
                        }
                    }
                }

                return out;
            }

            // Handle array form: could be features directly
            if (mapShapes instanceof JSONArray) {
                JSONArray arr = (JSONArray) mapShapes;

                // If it looks like GeoJSON features
                if (arr.length() > 0 && arr.optJSONObject(0) != null) {
                    // might be [ {type:"Feature", ...}, ...]
                    consumeFeaturesIntoFloors(arr, out);
                    return out;
                }
            }

            Log.d(TAG, "map_shapes exists but schema not recognized. type=" + mapShapes.getClass().getSimpleName());
        } catch (Exception e) {
            Log.e(TAG, "Failed parsing map_shapes", e);
        }

        return out;
    }

    private void consumeFeaturesIntoFloors(@Nullable JSONArray features,
                                           @NonNull Map<String, List<List<LatLng>>> out) throws Exception {
        if (features == null) return;

        // Keep seen floors in insertion order
        LinkedHashSet<String> seen = new LinkedHashSet<>();

        for (int i = 0; i < features.length(); i++) {
            JSONObject f = features.optJSONObject(i);
            if (f == null) continue;

            JSONObject props = f.optJSONObject("properties");
            String floor = extractFloorFromProperties(props);
            if (floor == null) floor = "unknown";

            JSONObject geom = f.optJSONObject("geometry");
            if (geom == null) continue;

            List<List<LatLng>> polys = geometryToPolygons(geom);
            if (polys.isEmpty()) continue;

            if (!out.containsKey(floor)) out.put(floor, new ArrayList<>());
            out.get(floor).addAll(polys);
            seen.add(floor);
        }

        Log.d(TAG, "Parsed floors from map_shapes: " + seen);
    }

    private void consumeFeaturesIntoFloorsWithFixedFloor(@Nullable JSONArray features,
                                                         @NonNull Map<String, List<List<LatLng>>> out,
                                                         @NonNull String floor) throws Exception {
        if (features == null) return;

        for (int i = 0; i < features.length(); i++) {
            JSONObject f = features.optJSONObject(i);
            if (f == null) continue;

            JSONObject geom = f.optJSONObject("geometry");
            if (geom == null) continue;

            List<List<LatLng>> polys = geometryToPolygons(geom);
            if (polys.isEmpty()) continue;

            if (!out.containsKey(floor)) out.put(floor, new ArrayList<>());
            out.get(floor).addAll(polys);
        }

        Log.d(TAG, "Parsed floor bucket: " + floor + " count=" + out.get(floor).size());
    }

    @Nullable
    private String extractFloorFromProperties(@Nullable JSONObject props) {
        if (props == null) return null;

        // Try common keys
        String f = props.optString("floor", null);
        if (f != null && !f.trim().isEmpty()) return f;

        f = props.optString("level", null);
        if (f != null && !f.trim().isEmpty()) return f;

        // Sometimes numeric (z)
        if (props.has("z")) return String.valueOf(props.optInt("z"));

        // Sometimes name includes floor
        f = props.optString("name", null);
        if (f != null && !f.trim().isEmpty()) return f;

        return null;
    }

    // =========================
    // GeoJSON geometry parsing
    // =========================

    @Nullable
    private List<LatLng> parseGeometryToLatLngs(@NonNull JSONObject geometry) {
        String type = geometry.optString("type", "");
        JSONArray coords = geometry.optJSONArray("coordinates");

        if ("Polygon".equalsIgnoreCase(type)) return parsePolygon(coords);
        if ("MultiPolygon".equalsIgnoreCase(type)) return parseMultiPolygon(coords);

        return null;
    }

    @NonNull
    private List<List<LatLng>> geometryToPolygons(@NonNull JSONObject geometry) {
        List<List<LatLng>> out = new ArrayList<>();

        String type = geometry.optString("type", "");
        JSONArray coords = geometry.optJSONArray("coordinates");

        if ("Polygon".equalsIgnoreCase(type)) {
            List<LatLng> p = parsePolygon(coords);
            if (p != null) out.add(p);
        } else if ("MultiPolygon".equalsIgnoreCase(type)) {
            // Add first polygon outer ring for each polygon entry
            if (coords != null) {
                for (int i = 0; i < coords.length(); i++) {
                    JSONArray poly = coords.optJSONArray(i);
                    if (poly == null) continue;
                    // poly[0] = outer ring
                    JSONArray ring = poly.optJSONArray(0);
                    if (ring == null) continue;
                    List<LatLng> pts = parseRing(ring);
                    if (pts != null) out.add(pts);
                }
            }
        }

        return out;
    }

    @Nullable
    private List<LatLng> parsePolygon(@Nullable JSONArray coordinates) {
        if (coordinates == null) return null;
        JSONArray ring = coordinates.optJSONArray(0);
        if (ring == null) return null;
        return parseRing(ring);
    }

    @Nullable
    private List<LatLng> parseMultiPolygon(@Nullable JSONArray coordinates) {
        if (coordinates == null) return null;
        JSONArray poly0 = coordinates.optJSONArray(0);
        if (poly0 == null) return null;
        JSONArray ring0 = poly0.optJSONArray(0);
        if (ring0 == null) return null;
        return parseRing(ring0);
    }

    @Nullable
    private List<LatLng> parseRing(@NonNull JSONArray ring) {
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
    private LatLng computeCentroid(@NonNull List<LatLng> pts) {
        double lat = 0.0;
        double lon = 0.0;
        for (LatLng p : pts) {
            lat += p.latitude;
            lon += p.longitude;
        }
        return new LatLng(lat / pts.size(), lon / pts.size());
    }

}


