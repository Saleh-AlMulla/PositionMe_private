package com.openpositioning.PositionMe.sensors;

import android.util.Log;

import com.google.android.gms.maps.model.LatLng;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.openpositioning.PositionMe.mapmatching.MapMatchingEngine;
import com.openpositioning.PositionMe.positioning.FusionManager;

/**
 * Manages WiFi scan result processing and WiFi-based positioning requests.
 *
 * <p>Implements {@link Observer} to receive updates from {@link WifiDataProcessor}.
 * On each WiFi scan result, it fires a positioning request and feeds the result
 * into both {@link FusionManager} and {@link MapMatchingEngine}.</p>
 *
 * @see WifiDataProcessor the observable that triggers WiFi scan updates
 * @see WiFiPositioning   the API client for WiFi-based positioning
 */
public class WifiPositionManager implements Observer {

    private static final String TAG = "WifiPositionManager";
    private static final String WIFI_FINGERPRINT = "wf";

    private final WiFiPositioning wiFiPositioning;
    private final TrajectoryRecorder recorder;
    private List<Wifi> wifiList;

    /**
     * Creates a new WifiPositionManager.
     *
     * @param wiFiPositioning WiFi positioning API client
     * @param recorder        trajectory recorder for writing WiFi fingerprints
     */
    public WifiPositionManager(WiFiPositioning wiFiPositioning,
                               TrajectoryRecorder recorder) {
        this.wiFiPositioning = wiFiPositioning;
        this.recorder = recorder;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Receives updates from {@link WifiDataProcessor}. Converts the raw object array
     * to a typed list, delegates fingerprint recording to {@link TrajectoryRecorder},
     * and triggers a WiFi positioning request.</p>
     */
    @Override
    public void update(Object[] wifiList) {
        this.wifiList = Stream.of(wifiList).map(o -> (Wifi) o).collect(Collectors.toList());
        recorder.addWifiFingerprint(this.wifiList);
        createWifiPositionRequestCallback();
    }

    /**
     * Creates a WiFi positioning request using the Volley callback pattern.
     * On success, feeds the result into both FusionManager and MapMatchingEngine.
     */
    private void createWifiPositionRequestCallback() {
        if (this.wifiList == null || this.wifiList.isEmpty()) {
            return;
        }
        try {
            JSONObject wifiAccessPoints = new JSONObject();
            for (Wifi data : this.wifiList) {
                wifiAccessPoints.put(String.valueOf(data.getBssid()), data.getLevel());
            }
            JSONObject wifiFingerPrint = new JSONObject();
            wifiFingerPrint.put(WIFI_FINGERPRINT, wifiAccessPoints);

            this.wiFiPositioning.request(wifiFingerPrint, new WiFiPositioning.VolleyCallback() {
                @Override
                public void onSuccess(LatLng wifiLocation, int floor) {
                    if (wifiLocation == null) return;

                    // Feed into FusionManager (map-unaware, auto-initialising filter)
                    FusionManager.getInstance().onWifi(
                            wifiLocation.latitude,
                            wifiLocation.longitude
                    );

                    // Feed into MapMatchingEngine (wall-aware filter)
                    // This was previously missing — WiFi corrections were never
                    // reaching the map matching engine.
                    MapMatchingEngine engine = SensorFusion.getInstance().getMapMatchingEngine();
                    if (engine != null && engine.isActive()) {
                        engine.updateWifi(
                                wifiLocation.latitude,
                                wifiLocation.longitude,
                                floor,
                                10.0f
                        );
                    }

                    // TODO: remove before submission
                    Log.d(TAG, "WiFi fix → lat=" + wifiLocation.latitude
                            + " lng=" + wifiLocation.longitude
                            + " floor=" + floor
                            + " mmEngine=" + (engine != null && engine.isActive()
                            ? "updated" : "not active"));
                }

                @Override
                public void onError(String message) {
                    Log.e(TAG, "WiFi positioning failed: " + message);
                }
            });
        } catch (JSONException e) {
            Log.e(TAG, "Error creating WiFi JSON: " + e);
        }
    }

    /**
     * Returns the user position obtained using WiFi positioning.
     *
     * @return {@link LatLng} corresponding to the user's position
     */
    public LatLng getLatLngWifiPositioning() {
        return this.wiFiPositioning.getWifiLocation();
    }

    /**
     * Returns the current floor the user is on, obtained using WiFi positioning.
     *
     * @return current floor number
     */
    public int getWifiFloor() {
        return this.wiFiPositioning.getFloor();
    }

    /**
     * Returns the most recent list of WiFi scan results.
     *
     * @return list of {@link Wifi} objects
     */
    public List<Wifi> getWifiList() {
        return this.wifiList;
    }
}