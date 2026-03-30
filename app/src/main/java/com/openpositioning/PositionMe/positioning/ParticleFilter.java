package com.openpositioning.PositionMe.positioning;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class ParticleFilter {

    private static class Particle {
        double east;
        double north;
        double weight;

        Particle(double east, double north, double weight) {
            this.east = east;
            this.north = north;
            this.weight = weight;
        }

        Particle copy() {
            return new Particle(east, north, weight);
        }
    }

    private static final double DEFAULT_INIT_SIGMA = 4.0;
    private static final double MOTION_NOISE_BASE = 0.12;
    private static final double MOTION_NOISE_STEP_SCALE = 0.08;
    private static final double STEP_NOISE_SIGMA = 0.05;
    private static final double HEADING_NOISE_SIGMA = 0.05;
    private static final double GNSS_SIGMA_MIN = 3.0;
    //private static final double WIFI_SIGMA_DEFAULT = 6.0;
    private static final double WIFI_SIGMA_DEFAULT = 10.0;

    private final Random rng = new Random(42);
    private final List<Particle> particles = new ArrayList<>();
    private final int particleCount;
    private boolean initialised = false;

    public ParticleFilter(int particleCount) {
        this.particleCount = particleCount;
    }

    public boolean isInitialised() {
        return initialised;
    }

    public void initialise(double east, double north, double sigmaMeters) {
        particles.clear();
        double sigma = Math.max(1.0, sigmaMeters > 0 ? sigmaMeters : DEFAULT_INIT_SIGMA);
        for (int i = 0; i < particleCount; i++) {
            particles.add(new Particle(
                    east + rng.nextGaussian() * sigma,
                    north + rng.nextGaussian() * sigma,
                    1.0 / particleCount
            ));
        }
        initialised = true;
    }

    public void predict(double stepLength, double headingRad) {
        if (!initialised || stepLength <= 0.05 || Double.isNaN(stepLength) || Double.isNaN(headingRad)) {
            return;
        }

        double motionSigma = MOTION_NOISE_BASE + MOTION_NOISE_STEP_SCALE * stepLength;

        for (Particle p : particles) {
            double noisyStep = stepLength + rng.nextGaussian() * STEP_NOISE_SIGMA;
            double noisyHeading = headingRad + rng.nextGaussian() * HEADING_NOISE_SIGMA;

            double dE = noisyStep * Math.sin(noisyHeading);
            double dN = noisyStep * Math.cos(noisyHeading);

            p.east += dE + rng.nextGaussian() * motionSigma;
            p.north += dN + rng.nextGaussian() * motionSigma;
        }
    }

    public void updateGnss(double obsEast, double obsNorth, float accuracyMeters) {
        double sigma = Math.max(GNSS_SIGMA_MIN, accuracyMeters);
        updateAbsolute(obsEast, obsNorth, sigma);
    }

    public void updateWifi(double obsEast, double obsNorth) {
        updateAbsolute(obsEast, obsNorth, WIFI_SIGMA_DEFAULT);
    }

    private void updateAbsolute(double obsEast, double obsNorth, double sigma) {
        if (!initialised) return;

        double sigma2 = sigma * sigma;
        double totalWeight = 0.0;

        for (Particle p : particles) {
            double dE = p.east - obsEast;
            double dN = p.north - obsNorth;
            double dist2 = dE * dE + dN * dN;
            double likelihood = Math.exp(-0.5 * dist2 / sigma2);
            p.weight *= Math.max(likelihood, 1e-12);
            totalWeight += p.weight;
        }

        if (totalWeight < 1e-20) {
            initialise(obsEast, obsNorth, sigma);
            return;
        }

        normalise(totalWeight);

        if (effectiveSampleSize() < particleCount * 0.5) {
            resampleSystematic();
        }
    }

    private void normalise(double totalWeight) {
        for (Particle p : particles) {
            p.weight /= totalWeight;
        }
    }

    private double effectiveSampleSize() {
        double sumSq = 0.0;
        for (Particle p : particles) {
            sumSq += p.weight * p.weight;
        }
        return sumSq <= 0.0 ? 0.0 : 1.0 / sumSq;
    }

    private void resampleSystematic() {
        List<Particle> resampled = new ArrayList<>(particleCount);
        double step = 1.0 / particleCount;
        double u0 = rng.nextDouble() * step;
        double cumulative = particles.get(0).weight;
        int i = 0;

        for (int m = 0; m < particleCount; m++) {
            double threshold = u0 + m * step;
            while (threshold > cumulative && i < particles.size() - 1) {
                i++;
                cumulative += particles.get(i).weight;
            }
            Particle picked = particles.get(i).copy();
            picked.weight = 1.0 / particleCount;
            resampled.add(picked);
        }

        particles.clear();
        particles.addAll(resampled);
    }

    public double[] estimate() {
        if (!initialised || particles.isEmpty()) return null;

        double east = 0.0;
        double north = 0.0;
        for (Particle p : particles) {
            east += p.east * p.weight;
            north += p.north * p.weight;
        }
        return new double[]{east, north};
    }

    public double getConfidence() {
        if (!initialised || particles.isEmpty()) return 0.0;
        double ess = effectiveSampleSize();
        return Math.max(0.0, Math.min(1.0, ess / particleCount));
    }
}
