package com.openpositioning.PositionMe.positioning;

import android.util.Log;

public class FusionManager {

    private static final String TAG = "FusionManager";
    private static final int PARTICLE_COUNT = 250;

    private static final FusionManager instance = new FusionManager();

    private ParticleFilter particleFilter;
    private CoordinateUtils coords;
    private boolean ready = false;

    private double[] smoothedLatLon = null;
    private static final double DISPLAY_SMOOTHING_ALPHA = 0.25;

    // Track initialisation source to prevent re-init from weaker source
    private boolean initialisedFromWifi = false;

    private FusionManager() {}

    public static FusionManager getInstance() {
        return instance;
    }

    public synchronized void reset() {
        particleFilter = null;
        coords = null;
        ready = false;
        smoothedLatLon = null;
        initialisedFromWifi = false;
    }

    public synchronized void onGnss(double lat, double lon, float accuracyMeters) {
        float rawAccuracy = accuracyMeters > 0 ? accuracyMeters : 8.0f;

        // Stricter threshold — indoor GNSS below 15m is rare
        if (rawAccuracy > 15.0f) {
            Log.d(TAG, "Ignoring GNSS fix due to poor accuracy: " + rawAccuracy);
            return;
        }

        float sigma = Math.max(6.0f, rawAccuracy * 1.5f);

        if (!ready) {
            coords = new CoordinateUtils(lat, lon);
            particleFilter = new ParticleFilter(PARTICLE_COUNT);
            particleFilter.initialise(0.0, 0.0, Math.max(sigma, 6.0f));
            ready = true;
            Log.d(TAG, "Initialised from GNSS: " + lat + ", " + lon + " acc=" + sigma);
            return;
        }

        double[] en = coords.toLocal(lat, lon);

        // Outlier gate: reject fixes far from current estimate
        double[] est = particleFilter.estimate();
        if (est != null) {
            double de = en[0] - est[0];
            double dn = en[1] - est[1];
            double dist = Math.sqrt(de * de + dn * dn);

            if (dist > Math.max(18.0, sigma * 2.5)) {
                Log.d(TAG, "Rejected GNSS outlier. Dist=" + dist + " sigma=" + sigma);
                return;
            }
        }

        particleFilter.updateGnss(en[0], en[1], sigma);
    }

    public synchronized void onWifi(double lat, double lon) {
        if (!ready) {
            // First fix — initialise from WiFi
            coords = new CoordinateUtils(lat, lon);
            particleFilter = new ParticleFilter(PARTICLE_COUNT);
            particleFilter.initialise(0.0, 0.0, 8.0);
            ready = true;
            initialisedFromWifi = true;
            Log.d(TAG, "Initialised from WiFi: " + lat + ", " + lon);
            return;
        }

        // If we initialised from GNSS and WiFi arrives soon after,
        // re-centre on WiFi since it's more accurate indoors
        if (!initialisedFromWifi && coords != null) {
            double[] en = coords.toLocal(lat, lon);
            double[] est = particleFilter.estimate();
            if (est != null) {
                double dist = Math.sqrt(Math.pow(en[0] - est[0], 2) + Math.pow(en[1] - est[1], 2));
                // If WiFi says we're far from GNSS init, re-centre once
                if (dist > 10.0) {
                    coords = new CoordinateUtils(lat, lon);
                    particleFilter = new ParticleFilter(PARTICLE_COUNT);
                    particleFilter.initialise(0.0, 0.0, 8.0);
                    initialisedFromWifi = true;
                    Log.d(TAG, "Re-centred from WiFi (GNSS was " + String.format("%.1f", dist) + "m off): " + lat + ", " + lon);
                    return;
                }
            }
            initialisedFromWifi = true; // Don't try re-centring again
        }

        double[] en = coords.toLocal(lat, lon);

        double[] est = particleFilter.estimate();
        if (est != null) {
            double de = en[0] - est[0];
            double dn = en[1] - est[1];
            double dist = Math.sqrt(de * de + dn * dn);

            if (dist > 15.0) {
                Log.d(TAG, "Rejected WiFi outlier. Dist=" + dist);
                return;
            }
        }

        particleFilter.updateWifi(en[0], en[1]);
    }

    public synchronized void onStep(double stepLen, double headingRad) {
        if (!ready || particleFilter == null) return;
        if (Double.isNaN(stepLen) || Double.isNaN(headingRad) || stepLen <= 0.05) return;
        particleFilter.predict(stepLen, headingRad);
    }

    public synchronized double[] getBestPosition() {
        if (!ready || particleFilter == null || coords == null) return null;

        double[] en = particleFilter.estimate();
        if (en == null) return null;

        double[] latLon = coords.toGlobal(en[0], en[1]);

        if (smoothedLatLon == null) {
            smoothedLatLon = latLon;
        } else {
            smoothedLatLon[0] =
                    DISPLAY_SMOOTHING_ALPHA * latLon[0] +
                            (1.0 - DISPLAY_SMOOTHING_ALPHA) * smoothedLatLon[0];
            smoothedLatLon[1] =
                    DISPLAY_SMOOTHING_ALPHA * latLon[1] +
                            (1.0 - DISPLAY_SMOOTHING_ALPHA) * smoothedLatLon[1];
        }

        return new double[]{smoothedLatLon[0], smoothedLatLon[1]};
    }

    public synchronized double[] getBestPositionLocal() {
        if (!ready || particleFilter == null) return null;
        return particleFilter.estimate();
    }

    public synchronized double getConfidence() {
        if (!ready || particleFilter == null) return 0.0;
        return particleFilter.getConfidence();
    }

    public synchronized boolean isReady() {
        return ready;
    }
}