package com.openpositioning.PositionMe.positioning;

public class EKFFilter {
    private double east, north;
    private double pEast, pNorth;

    private static final double Q      = 0.3;
    private static final double R_GNSS = 15.0;
    private static final double R_WIFI = 6.0;

    public EKFFilter(double initEast, double initNorth) {
        this.east   = initEast;
        this.north  = initNorth;
        this.pEast  = 100.0;
        this.pNorth = 100.0;
    }

    public void predict(double dE, double dN) {
        east   += dE;
        north  += dN;
        pEast  += Q;
        pNorth += Q;
    }

    public void update(double obsEast, double obsNorth, double R) {
        double kE = pEast  / (pEast  + R);
        double kN = pNorth / (pNorth + R);
        east   += kE * (obsEast  - east);
        north  += kN * (obsNorth - north);
        pEast  *= (1 - kE);
        pNorth *= (1 - kN);
    }

    public double getEast()  { return east;  }
    public double getNorth() { return north; }
    public static double getGnssNoise() { return R_GNSS; }
    public static double getWifiNoise() { return R_WIFI; }
}