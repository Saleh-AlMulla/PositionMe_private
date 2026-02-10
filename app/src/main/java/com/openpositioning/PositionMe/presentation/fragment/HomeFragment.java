package com.openpositioning.PositionMe.presentation.fragment;

import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;
import androidx.navigation.NavDirections;
import androidx.navigation.Navigation;
import androidx.preference.PreferenceManager;

import com.google.android.material.button.MaterialButton;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.openpositioning.PositionMe.BuildConfig;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.data.remote.FloorplanApiClient;
import com.openpositioning.PositionMe.presentation.activity.IndoorActivity;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONException;
import java.util.HashMap;
import java.util.Map;

/**
 * HomeFragment
 * - Hub screen for navigation
 * - Shows Google Map + GNSS status
 * - Step (d): requests nearby indoor floorplans using LIVE location + LIVE observed Wi-Fi AP MACs
 */
public class HomeFragment extends Fragment implements OnMapReadyCallback {

    // Interactive UI elements to navigate to other fragments
    private MaterialButton goToInfo;
    private Button start;
    private Button measurements;
    private Button files;
    private TextView gnssStatusTextView;
    private MaterialButton indoorButton;

    // For the map
    private GoogleMap mMap;
    private SupportMapFragment mapFragment;

    // ===== Step (d) dependencies =====
    private WifiManager wifiManager;
    private final FloorplanApiClient floorplanApiClient = new FloorplanApiClient();
    private static final String TAG = "HomeFragmentIndoor";
    private static final int REQ_LOCATION_FOR_INDOOR = 1010;

    // Step (d): map overlays for venues
    private final Map<Polygon, JSONObject> polygonToVenue = new HashMap<>();
    private final List<Polygon> venuePolygons = new ArrayList<>();
    private final List<Marker> venueMarkers = new ArrayList<>();

    public HomeFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    /**
     * Ensure the action bar is shown at the top of the screen. Set the title visible to Home.
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        ((AppCompatActivity) getActivity()).getSupportActionBar().show();
        View rootView = inflater.inflate(R.layout.fragment_home, container, false);
        getActivity().setTitle("Home");
        return rootView;
    }

    /**
     * Initialise UI elements and set onClick actions for the buttons.
     */
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // Sensor Info button
        goToInfo = view.findViewById(R.id.sensorInfoButton);
        goToInfo.setOnClickListener(v -> {
            NavDirections action = HomeFragmentDirections.actionHomeFragmentToInfoFragment();
            Navigation.findNavController(v).navigate(action);
        });

        // Start/Stop Recording button
        start = view.findViewById(R.id.startStopButton);
        start.setEnabled(!PreferenceManager.getDefaultSharedPreferences(getContext())
                .getBoolean("permanentDeny", false));
        start.setOnClickListener(v -> {
            Intent intent = new Intent(requireContext(), RecordingActivity.class);
            startActivity(intent);
            ((AppCompatActivity) getActivity()).getSupportActionBar().hide();
        });

        // Measurements button
        measurements = view.findViewById(R.id.measurementButton);
        measurements.setOnClickListener(v -> {
            NavDirections action = HomeFragmentDirections.actionHomeFragmentToMeasurementsFragment();
            Navigation.findNavController(v).navigate(action);
        });

        // Files button
        files = view.findViewById(R.id.filesButton);
        files.setOnClickListener(v -> {
            NavDirections action = HomeFragmentDirections.actionHomeFragmentToFilesFragment();
            Navigation.findNavController(v).navigate(action);
        });

        // TextView to display GNSS disabled message
        gnssStatusTextView = view.findViewById(R.id.gnssStatusTextView);

        // Map fragment
        mapFragment = (SupportMapFragment)
                getChildFragmentManager().findFragmentById(R.id.mapFragmentContainer);
        if (mapFragment != null) {
            mapFragment.getMapAsync(this);
        }

        // Wifi manager (observed APs)
        wifiManager = (WifiManager) requireContext()
                .getApplicationContext()
                .getSystemService(Context.WIFI_SERVICE);

        // Indoor button:
        // - Step (d) request nearby indoor maps (live lat/lon + live MACs)
        // - Then (optionally) open your IndoorActivity list UI
        indoorButton = view.findViewById(R.id.indoorButton);
        indoorButton.setOnClickListener(v -> {
            requestNearbyIndoorMaps();

            // Keep your existing screen (SSID/RSSI list) if you still want it
            Intent intent = new Intent(requireContext(), IndoorActivity.class);
            startActivity(intent);
        });
    }

    /**
     * Callback triggered when the Google Map is ready to be used.
     */
    @Override
    public void onMapReady(@NonNull GoogleMap googleMap) {
        mMap = googleMap;
        checkAndUpdatePermissions();
    }

    @Override
    public void onResume() {
        super.onResume();
        checkAndUpdatePermissions();
    }

    /**
     * Checks if GNSS/Location is enabled on the device.
     */
    private boolean isGnssEnabled() {
        LocationManager locationManager =
                (LocationManager) requireContext().getSystemService(Context.LOCATION_SERVICE);
        boolean gpsEnabled = locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
        boolean networkEnabled = locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
        return (gpsEnabled || networkEnabled);
    }

    /**
     * Move the map to the University of Edinburgh and display a message.
     */
    private void showEdinburghAndMessage(String message) {
        gnssStatusTextView.setText(message);
        gnssStatusTextView.setVisibility(View.VISIBLE);

        LatLng edinburghLatLng = new LatLng(55.944425, -3.188396);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(edinburghLatLng, 15f));
        mMap.addMarker(new MarkerOptions()
                .position(edinburghLatLng)
                .title("University of Edinburgh"));
    }

    private void checkAndUpdatePermissions() {

        if (mMap == null) {
            return;
        }

        boolean gnssEnabled = isGnssEnabled();
        if (gnssEnabled) {
            gnssStatusTextView.setVisibility(View.GONE);

            if (ActivityCompat.checkSelfPermission(
                    requireContext(), Manifest.permission.ACCESS_FINE_LOCATION)
                    == PackageManager.PERMISSION_GRANTED ||
                    ActivityCompat.checkSelfPermission(
                            requireContext(), Manifest.permission.ACCESS_COARSE_LOCATION)
                            == PackageManager.PERMISSION_GRANTED) {

                mMap.setMyLocationEnabled(true);

            } else {
                showEdinburghAndMessage("Permission not granted. Please enable in settings.");
            }
        } else {
            showEdinburghAndMessage("GNSS is disabled. Please enable in settings.");
        }
    }

    // ======================================================================
    // Step (d) - request nearby indoor maps using LIVE location + LIVE MACs
    // ======================================================================

    private void requestNearbyIndoorMaps() {

        // Wi-Fi scan results require location permission on Android.
        boolean fineGranted = ActivityCompat.checkSelfPermission(
                requireContext(), Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
        boolean coarseGranted = ActivityCompat.checkSelfPermission(
                requireContext(), Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;

        if (!fineGranted && !coarseGranted) {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, REQ_LOCATION_FOR_INDOOR);
            return;
        }

        // Get last known location (best effort; good enough for coursework).
        Location loc = getBestLastKnownLocation();
        if (loc == null) {
            Toast.makeText(requireContext(),
                    "Location unavailable. Enable location and try again.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        double lat = loc.getLatitude();
        double lon = loc.getLongitude();

        // Observed AP MACs (BSSID)
        List<String> macs = getObservedMacs();
        if (macs.isEmpty()) {
            Toast.makeText(requireContext(),
                    "No Wi-Fi access points detected. Try again in a different area.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        // If user API key isn't set, fail early with a readable message.
        if (BuildConfig.OPENPOSITIONING_API_KEY == null || BuildConfig.OPENPOSITIONING_API_KEY.trim().isEmpty()) {
            Toast.makeText(requireContext(),
                    "OpenPositioning API key missing. Add OPENPOSITIONING_API_KEY to secrets.properties.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        Toast.makeText(requireContext(),
                "Requesting indoor maps… (" + macs.size() + " APs)",
                Toast.LENGTH_SHORT).show();

        floorplanApiClient.requestNearbyFloorplans(
                BuildConfig.OPENPOSITIONING_API_KEY,
                lat,
                lon,
                macs,
                new FloorplanApiClient.ResultCallback() {
                    @Override
                    public void onSuccess(@NonNull String rawJson) {
                        requireActivity().runOnUiThread(() -> {
                            Log.d(TAG, "Floorplan response: " + rawJson);
                            renderVenuesOnMap(rawJson);

                            // Step 2 goal: confirm request works and handle empty result cleanly.
                            if ("[]".equals(rawJson.trim())) {
                                Toast.makeText(requireContext(),
                                        "No indoor venues found nearby (try at Uni).",
                                        Toast.LENGTH_LONG).show();
                            } else {
                                Toast.makeText(requireContext(),
                                        "Indoor venues received (next: draw outlines).",
                                        Toast.LENGTH_LONG).show();
                            }
                        });
                    }

                    @Override
                    public void onError(@NonNull String message, @NonNull Exception e) {
                        requireActivity().runOnUiThread(() -> {
                            Log.e(TAG, message, e);
                            Toast.makeText(requireContext(),
                                    message,
                                    Toast.LENGTH_LONG).show();
                        });
                    }
                }
        );
    }

    @Nullable
    private Location getBestLastKnownLocation() {
        LocationManager lm =
                (LocationManager) requireContext().getSystemService(Context.LOCATION_SERVICE);

        Location best = null;

        try {
            Location gps = lm.getLastKnownLocation(LocationManager.GPS_PROVIDER);
            Location net = lm.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);

            if (gps != null) best = gps;
            if (net != null && (best == null || net.getAccuracy() < best.getAccuracy())) best = net;

        } catch (SecurityException ignored) {
            // Permission checked before calling
        }

        return best;
    }

    @NonNull
    private List<String> getObservedMacs() {

        // Trigger a refresh (async). Even if it fails, cached results are still useful.
        try {
            wifiManager.startScan();
        } catch (Exception ignored) { }

        List<ScanResult> results;
        try {
            results = wifiManager.getScanResults();
        } catch (SecurityException e) {
            return new ArrayList<>();
        }

        // De-duplicate + cap to keep request small and stable
        Set<String> unique = new HashSet<>();
        for (ScanResult r : results) {
            if (r.BSSID != null && !r.BSSID.isEmpty()) {
                unique.add(r.BSSID);
            }
            if (unique.size() >= 15) break;
        }

        return new ArrayList<>(unique);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == REQ_LOCATION_FOR_INDOOR) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                requestNearbyIndoorMaps();
            } else {
                Toast.makeText(requireContext(),
                        "Location permission is required for Wi-Fi scanning.",
                        Toast.LENGTH_LONG).show();
            }
        }
    }
}


