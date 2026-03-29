package com.openpositioning.PositionMe.positioning;

import android.util.Log;

public class FusionManager {

    private static final String TAG = "FusionManager";
    private static final int PARTICLE_COUNT = 250;

    private static final FusionManager instance = new FusionManager();

    private ParticleFilter particleFilter;
    private CoordinateUtils coords;
    private boolean ready = false;

    private FusionManager() {}

    public static FusionManager getInstance() {
        return instance;
    }

    public synchronized void reset() {
        particleFilter = null;
        coords = null;
        ready = false;
    }

    public synchronized void onGnss(double lat, double lon, float accuracyMeters) {
        float sigma = accuracyMeters > 0 ? accuracyMeters : 8.0f;
        if (!ready) {
            coords = new CoordinateUtils(lat, lon);
            particleFilter = new ParticleFilter(PARTICLE_COUNT);
            particleFilter.initialise(0.0, 0.0, Math.max(sigma, 4.0f));
            ready = true;
            Log.d(TAG, "Initialised from GNSS: " + lat + ", " + lon + " acc=" + sigma);
            return;
        }

        double[] en = coords.toLocal(lat, lon);
        particleFilter.updateGnss(en[0], en[1], sigma);
    }

    public synchronized void onWifi(double lat, double lon) {
        if (!ready) {
            coords = new CoordinateUtils(lat, lon);
            particleFilter = new ParticleFilter(PARTICLE_COUNT);
            particleFilter.initialise(0.0, 0.0, 6.0);
            ready = true;
            Log.d(TAG, "Initialised from WiFi: " + lat + ", " + lon);
            return;
        }

        double[] en = coords.toLocal(lat, lon);
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
        return coords.toGlobal(en[0], en[1]);
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