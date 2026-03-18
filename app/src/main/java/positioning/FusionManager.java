package com.openpositioning.PositionMe.positioning;

import android.util.Log;

public class FusionManager {

    private static final FusionManager instance = new FusionManager();

    private EKFFilter ekf;
    private CoordinateUtils coords;
    private boolean ready = false;

    private FusionManager() {}

    public static FusionManager getInstance() {
        return instance;
    }

    public void onGnss(double lat, double lon) {
        if (!ready) {
            coords = new CoordinateUtils(lat, lon);
            ekf = new EKFFilter(0, 0);
            ready = true;
            Log.d("FusionManager", "Initialised at " + lat + ", " + lon);
            return;
        }
        double[] en = coords.toLocal(lat, lon);
        ekf.update(en[0], en[1], EKFFilter.getGnssNoise());
    }

    public void onWifi(double lat, double lon) {
        if (!ready) return;
        double[] en = coords.toLocal(lat, lon);
        ekf.update(en[0], en[1], EKFFilter.getWifiNoise());
    }

    public void onStep(double stepLen, double headingRad) {
        if (!ready) return;
        double dE = stepLen * Math.sin(headingRad);
        double dN = stepLen * Math.cos(headingRad);
        ekf.predict(dE, dN);
    }

    public double[] getBestPosition() {
        if (!ready) return null;
        return coords.toGlobal(ekf.getEast(), ekf.getNorth());
    }

    public boolean isReady() {
        return ready;
    }
}