package com.openpositioning.PositionMe.mapmatching;

import android.util.Log;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.data.remote.FloorplanApiClient;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Particle filter engine for indoor map matching.
 *
 * <p>Uses a set of weighted particles to estimate the user's position and floor.
 * Each particle is propagated using PDR displacement, then weighted against map
 * constraints (walls, stairs, lifts) and positioning observations (WiFi, GNSS).
 * Systematic resampling keeps the particle population healthy.</p>
 *
 * <h3>Map constraint types (from floorplan API):</h3>
 * <ul>
 *   <li><b>wall</b>  — impassable barriers; particles crossing them are penalised</li>
 *   <li><b>stairs</b> — floor transitions with horizontal movement</li>
 *   <li><b>lift</b>  — floor transitions without significant horizontal displacement</li>
 * </ul>
 *
 * <h3>Integration points:</h3>
 * <ul>
 *   <li>{@link com.openpositioning.PositionMe.sensors.SensorFusion} — owns the engine instance</li>
 *   <li>{@link com.openpositioning.PositionMe.sensors.SensorEventHandler} — calls
 *       {@link #predict(float, float, float)} on each PDR step</li>
 *   <li>{@link com.openpositioning.PositionMe.sensors.WifiPositionManager} — calls
 *       {@link #updateWifi(double, double, int, float)} on WiFi position updates</li>
 * </ul>
 *
 * @see Particle individual particle state
 */
public class MapMatchingEngine {

    private static final String TAG = "MapMatchingEngine";

    // -------------------------------------------------------------------------
    // Tuneable parameters
    // -------------------------------------------------------------------------

    /** Number of particles in the filter. */
    private static final int NUM_PARTICLES = 200;

    /** Standard deviation of noise added to PDR step length (metres). */
    private static final double PDR_NOISE_STEP = 0.15;

    /** Standard deviation of noise added to PDR heading (radians). */
    private static final double PDR_NOISE_HEADING = Math.toRadians(8);

    /** Standard deviation for WiFi observation likelihood (metres). */
    private static final double WIFI_SIGMA = 8.0;

    /** Standard deviation for GNSS observation likelihood (metres). */
    private static final double GNSS_SIGMA = 12.0;

    /**
     * Weight multiplier applied to particles that cross a wall.
     * Lower value = harsher penalty.
     */
    private static final double WALL_PENALTY = 0.001;

    /** Distance threshold (metres) to consider a particle near stairs or lift. */
    private static final double TRANSITION_PROXIMITY = 4.0;

    /**
     * Minimum elevation change from the last floor's baseline (metres) required
     * to trigger a floor transition check.
     */
    private static final double FLOOR_CHANGE_ELEVATION_THRESHOLD = 2.0;

    /** Effective sample size ratio below which resampling is triggered. */
    private static final double RESAMPLE_THRESHOLD = 0.5;

    /**
     * Spread radius (metres) for initial particle distribution.
     * Set to 6.0 to handle indoor GPS error which can easily be 5-10m.
     */
    private static final double INIT_SPREAD = 6.0;

    /**
     * Noise (metres) injected into each particle after resampling to maintain
     * diversity and prevent the filter collapsing to a single point.
     */
    private static final double RESAMPLE_JITTER = 0.5;

    /**
     * Maximum total horizontal distance (metres) accumulated while elevation is
     * changing that still classifies the transition as a lift (not stairs).
     */
    private static final double LIFT_HORIZONTAL_THRESHOLD = 1.5;

    // -------------------------------------------------------------------------
    // Coordinate conversion — metres per degree at Edinburgh (~55.92 N)
    // -------------------------------------------------------------------------
    private static final double METRES_PER_DEG_LAT = 111_320.0;
    private static final double METRES_PER_DEG_LNG =
            111_320.0 * Math.cos(Math.toRadians(55.92));

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private final List<Particle> particles = new ArrayList<>();
    private final Random random = new Random();

    private double refLat;
    private double refLng;
    private boolean initialised = false;

    private List<FloorplanApiClient.FloorShapes> floorShapes;

    private float currentElevation = 0f;
    private float elevationAtLastFloorChange = 0f;
    private float floorHeight = 4.0f;
    private int estimatedFloor = 0;
    private boolean enabled = false;

    // Stairs / lift detection
    private float lastStepLength = 0f;
    private float horizontalDuringElevationChange = 0f;
    private boolean inElevationChange = false;
    private float elevationChangeStartedAt = 0f;

    // Debug counters — TODO: remove before submission
    private int wallHitCount = 0;
    private int wallCheckCount = 0;
    private int predictCallCount = 0;

    // =========================================================================
    // Initialisation
    // =========================================================================

    /**
     * Initialises the particle filter around a starting position.
     *
     * @param lat         starting latitude (WGS84)
     * @param lng         starting longitude (WGS84)
     * @param floor       starting floor index
     * @param floorHeight floor height of the current building (metres)
     * @param shapes      per-floor vector shape data from floorplan API (may be null)
     */
    public void initialise(double lat, double lng, int floor,
                           float floorHeight, List<FloorplanApiClient.FloorShapes> shapes) {
        this.refLat = lat;
        this.refLng = lng;
        this.floorHeight = floorHeight;
        this.floorShapes = shapes;
        this.estimatedFloor = floor;
        this.currentElevation = 0f;
        this.elevationAtLastFloorChange = 0f;
        this.horizontalDuringElevationChange = 0f;
        this.inElevationChange = false;
        this.wallHitCount = 0;
        this.wallCheckCount = 0;
        this.predictCallCount = 0;

        particles.clear();
        for (int i = 0; i < NUM_PARTICLES; i++) {
            double px = random.nextGaussian() * INIT_SPREAD;
            double py = random.nextGaussian() * INIT_SPREAD;
            particles.add(new Particle(px, py, floor, 1.0 / NUM_PARTICLES));
        }

        this.initialised = true;
        this.enabled = true;

        // TODO: remove before submission
        Log.d(TAG, "initialise() called — lat=" + lat + " lng=" + lng
                + " floor=" + floor + " floorHeight=" + floorHeight
                + " shapes=" + (shapes == null ? "NULL" : shapes.size() + " floors"));
        logWallStats();
    }

    // TODO: remove before submission
    public void logWallStats() {
        if (floorShapes == null) {
            Log.d(TAG, "logWallStats: floorShapes is NULL — wall checking disabled");
            return;
        }
        for (int f = 0; f < floorShapes.size(); f++) {
            int wallSegments = 0;
            int stairFeatures = 0;
            int liftFeatures = 0;
            for (FloorplanApiClient.MapShapeFeature feature : floorShapes.get(f).getFeatures()) {
                String type = feature.getIndoorType();
                if ("wall".equals(type)) {
                    for (List<LatLng> part : feature.getParts()) {
                        wallSegments += part.size() - 1;
                    }
                } else if ("stairs".equals(type)) {
                    stairFeatures++;
                } else if ("lift".equals(type)) {
                    liftFeatures++;
                }
            }
            Log.d(TAG, "  Floor " + f + ": " + wallSegments + " wall segments, "
                    + stairFeatures + " stair features, " + liftFeatures + " lift features");
        }
    }

    /** Returns whether the engine has been initialised and is active. */
    public boolean isActive() {
        return initialised && enabled;
    }

    /** Enables or disables the engine without clearing particles. */
    public void setEnabled(boolean enabled) {
        // TODO: remove before submission
        Log.d(TAG, "setEnabled(" + enabled + ") — was " + this.enabled);
        this.enabled = enabled;
    }

    /** Resets the engine, clearing all particles and state. */
    public void reset() {
        // TODO: remove before submission
        Log.d(TAG, "reset() called");
        particles.clear();
        initialised = false;
        enabled = false;
        currentElevation = 0f;
        elevationAtLastFloorChange = 0f;
        estimatedFloor = 0;
        horizontalDuringElevationChange = 0f;
        inElevationChange = false;
        wallHitCount = 0;
        wallCheckCount = 0;
        predictCallCount = 0;
    }

    /** Updates the floor shapes used for wall/stair/lift constraints. */
    public void setFloorShapes(List<FloorplanApiClient.FloorShapes> shapes) {
        this.floorShapes = shapes;
        // TODO: remove before submission
        Log.d(TAG, "setFloorShapes() — "
                + (shapes == null ? "NULL" : shapes.size() + " floors"));
    }

    /** Updates the floor height for the current building. */
    public void setFloorHeight(float height) {
        this.floorHeight = height;
    }

    // =========================================================================
    // Prediction step — called on each PDR step
    // =========================================================================

    /**
     * Propagates all particles using the PDR displacement and applies map constraints.
     *
     * @param stepLength step length in metres from PDR
     * @param headingRad heading in radians (north = 0, clockwise)
     * @param elevation  current relative elevation from barometer (metres)
     */
    public void predict(float stepLength, float headingRad, float elevation) {
        // TODO: remove before submission
        predictCallCount++;
        if (predictCallCount <= 3 || predictCallCount % 20 == 0) {
            Log.d(TAG, "predict() #" + predictCallCount
                    + " — initialised=" + initialised
                    + " enabled=" + enabled
                    + " stepLen=" + String.format("%.2f", stepLength)
                    + " heading=" + String.format("%.1f", Math.toDegrees(headingRad)) + "deg"
                    + " elev=" + String.format("%.2f", elevation) + "m");
        }

        if (!isActive()) {
            Log.w(TAG, "predict() bailed — initialised=" + initialised
                    + " enabled=" + enabled);
            return;
        }

        this.currentElevation = elevation;
        this.lastStepLength = stepLength;

        for (Particle p : particles) {
            double noisyStep = stepLength + random.nextGaussian() * PDR_NOISE_STEP;
            double noisyHeading = headingRad + random.nextGaussian() * PDR_NOISE_HEADING;
            noisyStep = Math.max(0, Math.min(noisyStep, 2.0));

            double dx = noisyStep * Math.sin(noisyHeading);
            double dy = noisyStep * Math.cos(noisyHeading);

            double oldX = p.getX();
            double oldY = p.getY();
            double newX = oldX + dx;
            double newY = oldY + dy;

            wallCheckCount++;
            if (doesCrossWall(oldX, oldY, newX, newY, p.getFloor())) {
                p.setWeight(p.getWeight() * WALL_PENALTY);
                wallHitCount++;
            } else {
                p.setX(newX);
                p.setY(newY);
            }
        }

        // TODO: remove before submission — fires once per step (200 checks per step)
        if (wallCheckCount > 0 && wallCheckCount % 200 == 0) {
            Log.d(TAG, "Wall check #" + (wallCheckCount / 200)
                    + " — hits=" + wallHitCount + "/" + wallCheckCount
                    + " (" + String.format("%.1f", 100.0 * wallHitCount / wallCheckCount) + "%)"
                    + " floor=" + estimatedFloor
                    + " shapes=" + (floorShapes == null ? "NULL" : floorShapes.size() + " floors"));
            if (!particles.isEmpty()) {
                Particle p0 = particles.get(0);
                Log.d(TAG, "  Particle[0] xy=("
                        + String.format("%.2f", p0.getX()) + ", "
                        + String.format("%.2f", p0.getY()) + ") floor=" + p0.getFloor());
            }
        }

        checkFloorTransition();
        normaliseWeights();
        if (effectiveSampleSize() < RESAMPLE_THRESHOLD * NUM_PARTICLES) {
            resample();
        }
    }

    // =========================================================================
    // Observation updates
    // =========================================================================

    /**
     * Updates particle weights based on a WiFi position observation.
     *
     * @param lat       WiFi-estimated latitude
     * @param lng       WiFi-estimated longitude
     * @param wifiFloor WiFi-estimated floor number
     * @param accuracy  estimated accuracy in metres
     */
    public void updateWifi(double lat, double lng, int wifiFloor, float accuracy) {
        if (!isActive()) return;

        double obsX = (lng - refLng) * METRES_PER_DEG_LNG;
        double obsY = (lat - refLat) * METRES_PER_DEG_LAT;
        double sigma = Math.max(WIFI_SIGMA, accuracy);

        for (Particle p : particles) {
            double dist = Math.sqrt(Math.pow(p.getX() - obsX, 2)
                    + Math.pow(p.getY() - obsY, 2));
            double likelihood = gaussianLikelihood(dist, sigma);
            if (p.getFloor() != wifiFloor) likelihood *= 0.1;
            p.setWeight(p.getWeight() * likelihood);
        }

        normaliseWeights();
        if (effectiveSampleSize() < RESAMPLE_THRESHOLD * NUM_PARTICLES) {
            resample();
        }

        // TODO: remove before submission
        Log.d(TAG, "updateWifi() — obsXY=("
                + String.format("%.2f", obsX) + ", "
                + String.format("%.2f", obsY) + ")"
                + " floor=" + wifiFloor
                + " sigma=" + String.format("%.1f", sigma));
    }

    /**
     * Updates particle weights based on a GNSS position observation.
     *
     * @param lat      GNSS latitude
     * @param lng      GNSS longitude
     * @param accuracy GNSS accuracy in metres
     */
    public void updateGnss(double lat, double lng, float accuracy) {
        if (!isActive()) return;

        double obsX = (lng - refLng) * METRES_PER_DEG_LNG;
        double obsY = (lat - refLat) * METRES_PER_DEG_LAT;
        double sigma = Math.max(GNSS_SIGMA, accuracy);

        for (Particle p : particles) {
            double dist = Math.sqrt(Math.pow(p.getX() - obsX, 2)
                    + Math.pow(p.getY() - obsY, 2));
            double likelihood = gaussianLikelihood(dist, sigma);
            p.setWeight(p.getWeight() * likelihood);
        }

        normaliseWeights();
    }

    // =========================================================================
    // Floor transition logic
    // =========================================================================

    /**
     * Checks if the barometric elevation change warrants a floor transition,
     * and distinguishes between stairs and lifts based on accumulated horizontal
     * movement during the elevation change.
     */
    private void checkFloorTransition() {
        if (floorShapes == null || floorHeight <= 0) return;

        float elevationDeltaFromBaseline = currentElevation - elevationAtLastFloorChange;
        float elevationChangeMagnitude = Math.abs(elevationDeltaFromBaseline);

        if (elevationChangeMagnitude > 0.3f) {
            if (!inElevationChange) {
                inElevationChange = true;
                elevationChangeStartedAt = currentElevation;
                horizontalDuringElevationChange = 0f;
            }
            horizontalDuringElevationChange += lastStepLength;
        } else {
            inElevationChange = false;
            horizontalDuringElevationChange = 0f;
        }

        if (elevationChangeMagnitude < FLOOR_CHANGE_ELEVATION_THRESHOLD) return;

        int elevationFloorDelta = (int) Math.round(elevationDeltaFromBaseline / floorHeight);
        if (elevationFloorDelta == 0) return;

        int targetFloor = clampFloor(estimatedFloor + elevationFloorDelta);
        boolean isLift = horizontalDuringElevationChange < LIFT_HORIZONTAL_THRESHOLD;

        for (Particle p : particles) {
            if (p.getFloor() == targetFloor) continue;

            boolean nearTransition = isNearStairsOrLift(p.getX(), p.getY(), p.getFloor());

            if (nearTransition) {
                p.setFloor(targetFloor);
                if (isLift) {
                    p.setWeight(p.getWeight() * 1.2);
                } else {
                    p.setX(p.getX() + random.nextGaussian() * 1.0);
                    p.setY(p.getY() + random.nextGaussian() * 1.0);
                    p.setWeight(p.getWeight() * 1.2);
                }
            } else {
                p.setWeight(p.getWeight() * 0.5);
            }
        }

        // TODO: remove before submission
        Log.d(TAG, "Floor transition → floor " + targetFloor
                + " via " + (isLift ? "LIFT" : "STAIRS")
                + " horiz=" + String.format("%.2f", horizontalDuringElevationChange) + "m"
                + " elevDelta=" + String.format("%.2f", elevationDeltaFromBaseline) + "m");

        this.estimatedFloor = targetFloor;
        this.elevationAtLastFloorChange = currentElevation;
        this.horizontalDuringElevationChange = 0f;
        this.inElevationChange = false;
    }

    /**
     * Checks if a position is near stairs or lift features on the given floor.
     */
    private boolean isNearStairsOrLift(double x, double y, int floor) {
        if (floorShapes == null || floor < 0 || floor >= floorShapes.size()) return false;

        FloorplanApiClient.FloorShapes floorData = floorShapes.get(floor);
        for (FloorplanApiClient.MapShapeFeature feature : floorData.getFeatures()) {
            String type = feature.getIndoorType();
            if ("stairs".equals(type) || "lift".equals(type)) {
                for (List<LatLng> part : feature.getParts()) {
                    for (LatLng point : part) {
                        double fx = (point.longitude - refLng) * METRES_PER_DEG_LNG;
                        double fy = (point.latitude - refLat) * METRES_PER_DEG_LAT;
                        double dist = Math.sqrt(Math.pow(x - fx, 2) + Math.pow(y - fy, 2));
                        if (dist < TRANSITION_PROXIMITY) return true;
                    }
                }
            }
        }
        return false;
    }

    /** Clamps a floor index to valid bounds given the loaded floor shapes. */
    private int clampFloor(int floor) {
        if (floorShapes == null || floorShapes.isEmpty()) return floor;
        return Math.max(0, Math.min(floor, floorShapes.size() - 1));
    }

    // =========================================================================
    // Wall collision detection
    // =========================================================================

    /**
     * Tests whether moving from (x1,y1) to (x2,y2) crosses any wall segment
     * on the given floor. Coordinates are in local ENU metres.
     */
    private boolean doesCrossWall(double x1, double y1, double x2, double y2, int floor) {
        if (floorShapes == null || floor < 0 || floor >= floorShapes.size()) return false;

        FloorplanApiClient.FloorShapes floorData = floorShapes.get(floor);
        for (FloorplanApiClient.MapShapeFeature feature : floorData.getFeatures()) {
            if (!"wall".equals(feature.getIndoorType())) continue;

            for (List<LatLng> part : feature.getParts()) {
                int size = part.size();
                for (int i = 0; i < size - 1; i++) {
                    LatLng a = part.get(i);
                    LatLng b = part.get(i + 1);

                    double ax = (a.longitude - refLng) * METRES_PER_DEG_LNG;
                    double ay = (a.latitude - refLat) * METRES_PER_DEG_LAT;
                    double bx = (b.longitude - refLng) * METRES_PER_DEG_LNG;
                    double by = (b.latitude - refLat) * METRES_PER_DEG_LAT;

                    if (segmentsIntersect(x1, y1, x2, y2, ax, ay, bx, by)) {
                        return true;
                    }
                }

                // Check closing edge for polygon rings, guarding against zero-length
                // segments (standard GeoJSON closed rings repeat first point as last)
                if (size >= 3) {
                    String geoType = feature.getGeometryType();
                    if ("MultiPolygon".equals(geoType) || "Polygon".equals(geoType)) {
                        LatLng first = part.get(0);
                        LatLng last = part.get(size - 1);
                        if (first.latitude == last.latitude
                                && first.longitude == last.longitude) continue;

                        double fx = (first.longitude - refLng) * METRES_PER_DEG_LNG;
                        double fy = (first.latitude - refLat) * METRES_PER_DEG_LAT;
                        double lx = (last.longitude - refLng) * METRES_PER_DEG_LNG;
                        double ly = (last.latitude - refLat) * METRES_PER_DEG_LAT;

                        if (segmentsIntersect(x1, y1, x2, y2, lx, ly, fx, fy)) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /**
     * Tests if two line segments (p1 to p2) and (p3 to p4) intersect using the
     * cross-product orientation method.
     */
    private static boolean segmentsIntersect(double p1x, double p1y, double p2x, double p2y,
                                             double p3x, double p3y, double p4x, double p4y) {
        double d1 = crossProduct(p3x, p3y, p4x, p4y, p1x, p1y);
        double d2 = crossProduct(p3x, p3y, p4x, p4y, p2x, p2y);
        double d3 = crossProduct(p1x, p1y, p2x, p2y, p3x, p3y);
        double d4 = crossProduct(p1x, p1y, p2x, p2y, p4x, p4y);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
                && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }

        if (d1 == 0 && onSegment(p3x, p3y, p4x, p4y, p1x, p1y)) return true;
        if (d2 == 0 && onSegment(p3x, p3y, p4x, p4y, p2x, p2y)) return true;
        if (d3 == 0 && onSegment(p1x, p1y, p2x, p2y, p3x, p3y)) return true;
        if (d4 == 0 && onSegment(p1x, p1y, p2x, p2y, p4x, p4y)) return true;

        return false;
    }

    /** Cross product of vectors (b-a) x (c-a). */
    private static double crossProduct(double ax, double ay, double bx, double by,
                                       double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    /** Checks if point (px,py) lies on segment (ax,ay) to (bx,by), assuming collinearity. */
    private static boolean onSegment(double ax, double ay, double bx, double by,
                                     double px, double py) {
        return Math.min(ax, bx) <= px && px <= Math.max(ax, bx)
                && Math.min(ay, by) <= py && py <= Math.max(ay, by);
    }

    // =========================================================================
    // Resampling
    // =========================================================================

    /**
     * Calculates the effective sample size (ESS) of the particle population.
     * ESS = 1 / sum(wi squared). Low ESS indicates weight degeneracy.
     */
    private double effectiveSampleSize() {
        double sumSq = 0;
        for (Particle p : particles) sumSq += p.getWeight() * p.getWeight();
        return (sumSq > 0) ? 1.0 / sumSq : 0;
    }

    /**
     * Systematic resampling: selects particles proportional to their weights,
     * replaces the population with copies reset to uniform weight, then injects
     * small Gaussian jitter ({@link #RESAMPLE_JITTER}) to maintain diversity
     * and prevent the filter collapsing to a single point.
     */
    private void resample() {
        int n = particles.size();
        if (n == 0) return;

        double[] cumulative = new double[n];
        cumulative[0] = particles.get(0).getWeight();
        for (int i = 1; i < n; i++) {
            cumulative[i] = cumulative[i - 1] + particles.get(i).getWeight();
        }

        List<Particle> newParticles = new ArrayList<>(n);
        double step = 1.0 / n;
        double start = random.nextDouble() * step;

        int idx = 0;
        for (int i = 0; i < n; i++) {
            double target = start + i * step;
            while (idx < n - 1 && cumulative[idx] < target) idx++;
            Particle copy = particles.get(idx).copy();
            copy.setWeight(1.0 / n);
            newParticles.add(copy);
        }

        // Inject diversity jitter to prevent full collapse after aggressive resampling.
        // Without this, all surviving particles occupy the same XY and the filter
        // behaves like a single point tracker.
        for (Particle p : newParticles) {
            p.setX(p.getX() + random.nextGaussian() * RESAMPLE_JITTER);
            p.setY(p.getY() + random.nextGaussian() * RESAMPLE_JITTER);
        }

        particles.clear();
        particles.addAll(newParticles);
    }

    /**
     * Normalises all particle weights to sum to 1.
     * Resets to uniform if weights have collapsed entirely.
     */
    private void normaliseWeights() {
        double sum = 0;
        for (Particle p : particles) sum += p.getWeight();
        if (sum <= 0) {
            Log.w(TAG, "Weight collapse — resetting to uniform weights");
            double uniform = 1.0 / particles.size();
            for (Particle p : particles) p.setWeight(uniform);
            return;
        }
        for (Particle p : particles) p.setWeight(p.getWeight() / sum);
    }

    // =========================================================================
    // Output
    // =========================================================================

    /**
     * Returns the weighted mean position of all particles as a LatLng.
     *
     * @return best-estimate position, or null if not initialised
     */
    public LatLng getEstimatedPosition() {
        if (!isActive() || particles.isEmpty()) return null;

        double weightedX = 0, weightedY = 0;
        for (Particle p : particles) {
            weightedX += p.getX() * p.getWeight();
            weightedY += p.getY() * p.getWeight();
        }

        double lat = refLat + weightedY / METRES_PER_DEG_LAT;
        double lng = refLng + weightedX / METRES_PER_DEG_LNG;
        return new LatLng(lat, lng);
    }

    /**
     * Returns the weighted-majority floor estimate from the particle distribution.
     *
     * @return estimated floor index
     */
    public int getEstimatedFloor() {
        if (!isActive() || particles.isEmpty()) return estimatedFloor;

        java.util.Map<Integer, Double> floorWeights = new java.util.HashMap<>();
        for (Particle p : particles) {
            floorWeights.merge(p.getFloor(), p.getWeight(), Double::sum);
        }

        int bestFloor = estimatedFloor;
        double bestWeight = -1;
        for (java.util.Map.Entry<Integer, Double> entry : floorWeights.entrySet()) {
            if (entry.getValue() > bestWeight) {
                bestWeight = entry.getValue();
                bestFloor = entry.getKey();
            }
        }

        this.estimatedFloor = bestFloor;
        return bestFloor;
    }

    /**
     * Returns the estimated position as local XY coordinates (metres from reference).
     *
     * @return float[2] with {x, y}, or null if not initialised
     */
    public float[] getEstimatedXY() {
        if (!isActive() || particles.isEmpty()) return null;

        double weightedX = 0, weightedY = 0;
        for (Particle p : particles) {
            weightedX += p.getX() * p.getWeight();
            weightedY += p.getY() * p.getWeight();
        }
        return new float[]{(float) weightedX, (float) weightedY};
    }

    /**
     * Returns the current particle list as a snapshot (copies).
     * Useful for debug visualisation.
     */
    public List<Particle> getParticles() {
        List<Particle> snapshot = new ArrayList<>(particles.size());
        for (Particle p : particles) snapshot.add(p.copy());
        return snapshot;
    }

    // =========================================================================
    // Utility
    // =========================================================================

    /** Gaussian likelihood: exp(-d squared / (2 * sigma squared)). */
    private static double gaussianLikelihood(double distance, double sigma) {
        return Math.exp(-(distance * distance) / (2.0 * sigma * sigma));
    }
}