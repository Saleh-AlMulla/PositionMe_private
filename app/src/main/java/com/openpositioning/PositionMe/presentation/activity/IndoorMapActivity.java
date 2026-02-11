package com.openpositioning.PositionMe.presentation.activity;

import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.PolygonOptions;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.utils.IndoorSelectionStore;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/**
 * IndoorMapActivity
 * Shows the selected venue on its own map screen.
 * Today: draw outline only (your API response contains outline GeoJSON string).
 * Later: add floors + floorplan overlays when you have that endpoint.
 */
public class IndoorMapActivity extends AppCompatActivity implements OnMapReadyCallback {

    private static final String TAG = "IndoorMapActivity";
    private GoogleMap mMap;
    private JSONObject venue;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_indoor_map);

        // Pull the last selected venue from SharedPreferences
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

        String title = venue.optString("name", "Indoor Venue");
        setTitle("Indoor: " + title);

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

        List<LatLng> outline = extractOutlineLatLngs(venue);
        if (outline == null || outline.size() < 3) {
            Toast.makeText(this, "No outline available for this venue.", Toast.LENGTH_LONG).show();
            return;
        }

        mMap.addPolygon(new PolygonOptions().addAll(outline));

        // Zoom to venue (first point is good enough)
        LatLng first = outline.get(0);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(first, 19f));

        Toast.makeText(this, "Venue loaded. (Floorplans next)", Toast.LENGTH_SHORT).show();
    }

    // ===== Outline parsing for your current response shape =====

    @Nullable
    private List<LatLng> extractOutlineLatLngs(@NonNull JSONObject venueObj) {
        try {
            // API gives outline as a STRING containing FeatureCollection GeoJSON
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
                return parseMultiPolygon(coords);
            } else if ("Polygon".equalsIgnoreCase(type)) {
                return parsePolygon(coords);
            }
        } catch (Exception e) {
            Log.e(TAG, "Outline parse failed", e);
        }
        return null;
    }

    @Nullable
    private List<LatLng> parsePolygon(@Nullable JSONArray coordinates) {
        if (coordinates == null) return null;
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

    @Nullable
    private List<LatLng> parseMultiPolygon(@Nullable JSONArray coordinates) {
        if (coordinates == null) return null;

        // MultiPolygon: coordinates[0][0] is outer ring of first polygon
        JSONArray poly0 = coordinates.optJSONArray(0);
        if (poly0 == null) return null;

        JSONArray ring0 = poly0.optJSONArray(0);
        if (ring0 == null) return null;

        List<LatLng> pts = new ArrayList<>();
        for (int i = 0; i < ring0.length(); i++) {
            JSONArray pair = ring0.optJSONArray(i);
            if (pair == null || pair.length() < 2) continue;

            double lon = pair.optDouble(0, Double.NaN);
            double lat = pair.optDouble(1, Double.NaN);
            if (!Double.isNaN(lat) && !Double.isNaN(lon)) pts.add(new LatLng(lat, lon));
        }
        return pts.size() >= 3 ? pts : null;
    }
}
