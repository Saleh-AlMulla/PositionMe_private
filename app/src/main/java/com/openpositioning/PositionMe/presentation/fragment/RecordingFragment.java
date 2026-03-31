package com.openpositioning.PositionMe.presentation.fragment;

import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import android.view.animation.AlphaAnimation;
import android.view.animation.Animation;
import android.view.animation.LinearInterpolator;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import com.google.android.material.button.MaterialButton;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.preference.PreferenceManager;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.sensors.SensorTypes;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.google.android.gms.maps.model.LatLng;

import java.util.ArrayList;
import java.util.List;

import android.widget.Toast;
import com.openpositioning.PositionMe.positioning.FusionManager;

/**
 * Fragment responsible for managing the recording process of trajectory data.
 * <p>
 * The RecordingFragment serves as the interface for users to initiate, monitor, and
 * complete trajectory recording. It integrates sensor fusion data to track user movement
 * and updates a map view in real time.
 * <p>
 * Position display strategy:
 * <ol>
 *   <li>Primary: MapMatchingEngine — wall-constrained particle filter with floor detection.
 *       EMA-smoothed output with lost-filter recovery that resets the EMA on reinitialisation
 *       to prevent trace smearing outside the building.</li>
 *   <li>Fallback: FusionManager — smooth GPS+WiFi+PDR Kalman filter, used when
 *       MapMatchingEngine has not yet been initialised.</li>
 *   <li>Last resort: raw PDR position.</li>
 * </ol>
 * The orange trace (marker + polyline) always reflects one consistent source per cycle.
 * The red trace shows raw step-counting only, allowing visual comparison.
 *
 * @see TrajectoryMapFragment The map fragment displaying the recorded trajectory.
 * @see RecordingActivity The activity managing the recording workflow.
 * @see SensorFusion Handles sensor data collection.
 */
public class RecordingFragment extends Fragment {

    // TODO: remove before submission
    private static final String TAG = "RecordingFragment";

    // UI elements
    private MaterialButton completeButton, cancelButton;
    private ImageView recIcon;
    private ProgressBar timeRemaining;
    private TextView elevation, distanceTravelled, gnssError;

    // App settings
    private SharedPreferences settings;

    // Sensor & data logic
    private SensorFusion sensorFusion;
    private Handler refreshDataHandler;
    private CountDownTimer autoStop;

    // Distance tracking
    private float distance = 0f;
    private float previousPosX = 0f;
    private float previousPosY = 0f;

    private LatLng rawPdrPosition = null;

    // References to the child map fragment
    private TrajectoryMapFragment trajectoryMapFragment;
    private int uiUpdateCount = 0;

    private LatLng lastDisplayedGnssObservation = null;
    private long lastDisplayedGnssTimeMs = 0L;
    private static final long GNSS_DISPLAY_MIN_INTERVAL_MS = 1500L;
    private static final double GNSS_DISPLAY_MIN_DISTANCE_M = 3.0;

    private LatLng lastDisplayedWifiObservation = null;
    private long lastDisplayedWifiTimeMs = 0L;
    private static final long WIFI_DISPLAY_MIN_INTERVAL_MS = 1500L;
    private static final double WIFI_DISPLAY_MIN_DISTANCE_M = 2.0;

    private Float smoothedHeadingDeg = null;
    private static final float HEADING_SMOOTHING_ALPHA = 0.20f;

    private final Runnable refreshDataTask = new Runnable() {
        @Override
        public void run() {
            updateUIandPosition();
            refreshDataHandler.postDelayed(refreshDataTask, 200);
        }
    };

    public RecordingFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.sensorFusion = SensorFusion.getInstance();
        Context context = requireActivity();
        this.settings = PreferenceManager.getDefaultSharedPreferences(context);
        this.refreshDataHandler = new Handler();
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_recording, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view,
                              @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        trajectoryMapFragment = (TrajectoryMapFragment)
                getChildFragmentManager().findFragmentById(R.id.trajectoryMapFragmentContainer);

        if (trajectoryMapFragment == null) {
            trajectoryMapFragment = new TrajectoryMapFragment();
            getChildFragmentManager()
                    .beginTransaction()
                    .replace(R.id.trajectoryMapFragmentContainer, trajectoryMapFragment)
                    .commit();
        }

        elevation = view.findViewById(R.id.currentElevation);
        distanceTravelled = view.findViewById(R.id.currentDistanceTraveled);
        gnssError = view.findViewById(R.id.gnssError);

        completeButton = view.findViewById(R.id.stopButton);
        cancelButton = view.findViewById(R.id.cancelButton);
        recIcon = view.findViewById(R.id.redDot);
        timeRemaining = view.findViewById(R.id.timeRemainingBar);
        view.findViewById(R.id.btn_test_point).setOnClickListener(v -> onAddTestPoint());

        gnssError.setVisibility(View.GONE);
        elevation.setText(getString(R.string.elevation, "0"));
        distanceTravelled.setText(getString(R.string.meter, "0"));

        completeButton.setOnClickListener(v -> {
            if (autoStop != null) autoStop.cancel();
            sensorFusion.stopRecording();
            ((RecordingActivity) requireActivity()).showCorrectionScreen();
        });

        cancelButton.setOnClickListener(v -> {
            AlertDialog dialog = new AlertDialog.Builder(requireActivity())
                    .setTitle("Confirm Cancel")
                    .setMessage("Are you sure you want to cancel the recording? Your progress will be lost permanently!")
                    .setNegativeButton("Yes", (dialogInterface, which) -> {
                        sensorFusion.stopRecording();
                        if (autoStop != null) autoStop.cancel();
                        requireActivity().onBackPressed();
                    })
                    .setPositiveButton("No", (dialogInterface, which) -> dialogInterface.dismiss())
                    .create();

            dialog.setOnShowListener(dialogInterface -> {
                Button negativeButton = dialog.getButton(AlertDialog.BUTTON_NEGATIVE);
                negativeButton.setTextColor(Color.RED);
            });

            dialog.show();
        });

        blinkingRecordingIcon();

        if (this.settings.getBoolean("split_trajectory", false)) {
            long limit = this.settings.getInt("split_duration", 30) * 60000L;
            timeRemaining.setMax((int) (limit / 1000));
            timeRemaining.setProgress(0);
            timeRemaining.setScaleY(3f);

            autoStop = new CountDownTimer(limit, 1000) {
                @Override
                public void onTick(long millisUntilFinished) {
                    timeRemaining.incrementProgressBy(1);
                    updateUIandPosition();
                }

                @Override
                public void onFinish() {
                    sensorFusion.stopRecording();
                    ((RecordingActivity) requireActivity()).showCorrectionScreen();
                }
            }.start();
        } else {
            refreshDataHandler.post(refreshDataTask);
        }
    }

    private void onAddTestPoint() {
        if (trajectoryMapFragment == null) return;

        LatLng cur = trajectoryMapFragment.getCurrentLocation();
        if (cur == null) {
            Toast.makeText(requireContext(),
                    "I haven't gotten my current location yet, let me take a couple of steps/wait for the map to load.",
                    Toast.LENGTH_SHORT).show();
            return;
        }

        int idx = ++testPointIndex;
        long ts = System.currentTimeMillis();
        testPoints.add(new TestPoint(idx, ts, cur.latitude, cur.longitude));
        sensorFusion.addTestPointToProto(ts, cur.latitude, cur.longitude);
        trajectoryMapFragment.addTestPointMarker(idx, ts, cur);
    }

    /**
     * Updates the UI with sensor data and passes map updates to TrajectoryMapFragment.
     *
     * <p>Position source priority each cycle (~200ms):</p>
     * <ol>
     *   <li>MapMatchingEngine: wall-constrained particle filter — primary source.
     *       EMA-smoothed, resets cleanly on lost-filter recovery.</li>
     *   <li>FusionManager: Kalman-filtered GPS+WiFi+PDR — fallback before
     *       MapMatchingEngine initialises.</li>
     *   <li>Raw PDR: fallback of last resort.</li>
     * </ol>
     *
     * <p>The same {@code fusedLocation} variable is used for both
     * {@link TrajectoryMapFragment#updateUserLocation} (marker) and
     * {@link TrajectoryMapFragment#updatePdrObservation} (observation dot),
     * ensuring marker and polyline are always in sync.</p>
     */
    private void updateUIandPosition() {
        float[] pdrValues = sensorFusion.getSensorValueMap().get(SensorTypes.PDR);
        if (pdrValues == null) return;

        uiUpdateCount++;

        // Distance
        distance += Math.sqrt(Math.pow(pdrValues[0] - previousPosX, 2)
                + Math.pow(pdrValues[1] - previousPosY, 2));
        distanceTravelled.setText(getString(R.string.meter, String.format("%.2f", distance)));

        // Elevation
        float elevationVal = sensorFusion.getElevation();
        elevation.setText(getString(R.string.elevation, String.format("%.1f", elevationVal)));

        com.openpositioning.PositionMe.mapmatching.MapMatchingEngine mmEngine =
                sensorFusion.getMapMatchingEngine();

        // ── Wait for MapMatchingEngine before drawing anything ────────────────
        // This ensures both traces start at the manually dragged position,
        // not the GPS auto-fix. Before the engine is active, we still track
        // PDR accumulation so the first step delta is correct when we do start.
        if (mmEngine == null || !mmEngine.isActive()) {
            previousPosX = pdrValues[0];
            previousPosY = pdrValues[1];
            return;
        }
        // ─────────────────────────────────────────────────────────────────────

        // Initialise raw PDR position from engine's start (manually dragged point).
        // Only done once — first time engine is active.
        if (rawPdrPosition == null) {
            LatLng mmStart = mmEngine.getEstimatedPosition();
            if (mmStart != null) {
                rawPdrPosition = mmStart;
            }
        }

        // Advance raw PDR by step delta
        if (rawPdrPosition != null) {
            rawPdrPosition = UtilFunctions.calculateNewPos(
                    rawPdrPosition,
                    new float[]{ pdrValues[0] - previousPosX, pdrValues[1] - previousPosY }
            );
        }

        // Red polyline — raw step-counting only, no corrections
        if (rawPdrPosition != null && trajectoryMapFragment != null) {
            trajectoryMapFragment.addRawPdrPoint(rawPdrPosition);
        }

        // ── Position source selection ──────────────────────────────────────────
        LatLng fusedLocation = mmEngine.getEstimatedPosition();
        String positionSource = "MapMatchingEngine";

        if (fusedLocation == null) {
            double[] fusedPos = FusionManager.getInstance().getBestPosition();
            if (fusedPos != null) {
                fusedLocation = new LatLng(fusedPos[0], fusedPos[1]);
                positionSource = "FusionManager";
            }
        }
        // ─────────────────────────────────────────────────────────────────────

        // TODO: remove before submission
        if (uiUpdateCount % 5 == 0) {
            Log.d(TAG, "Position source=" + positionSource
                    + " | mmEngine.isActive=" + mmEngine.isActive()
                    + " | fusedLoc=" + (fusedLocation == null ? "null"
                    : String.format("(%.5f, %.5f)",
                    fusedLocation.latitude, fusedLocation.longitude)));
        }

        float headingDeg = smoothHeadingDeg(
                (float) Math.toDegrees(sensorFusion.passOrientation()));

        if (fusedLocation != null && trajectoryMapFragment != null) {
            trajectoryMapFragment.updateUserLocation(fusedLocation, headingDeg);
            trajectoryMapFragment.updatePdrObservation(fusedLocation);
        } else if (rawPdrPosition != null && trajectoryMapFragment != null) {
            trajectoryMapFragment.updateUserLocation(rawPdrPosition, headingDeg);
            trajectoryMapFragment.updatePdrObservation(rawPdrPosition);
        }

        long now = System.currentTimeMillis();

        // GNSS markers
        float[] gnss = sensorFusion.getSensorValueMap().get(SensorTypes.GNSSLATLONG);
        if (gnss != null && trajectoryMapFragment != null) {
            if (trajectoryMapFragment.isGnssEnabled()) {
                LatLng gnssLocation = new LatLng(gnss[0], gnss[1]);
                LatLng currentLoc = trajectoryMapFragment.getCurrentLocation();
                if (currentLoc != null) {
                    double errorDist = UtilFunctions.distanceBetweenPoints(
                            currentLoc, gnssLocation);
                    gnssError.setVisibility(View.VISIBLE);
                    gnssError.setText(String.format(
                            getString(R.string.gnss_error) + "%.2fm", errorDist));
                }
                if (shouldDisplayObservation(
                        gnssLocation, lastDisplayedGnssObservation,
                        now - lastDisplayedGnssTimeMs,
                        GNSS_DISPLAY_MIN_INTERVAL_MS, GNSS_DISPLAY_MIN_DISTANCE_M)) {
                    trajectoryMapFragment.updateGNSS(gnssLocation);
                    lastDisplayedGnssObservation = gnssLocation;
                    lastDisplayedGnssTimeMs = now;
                }
            } else {
                gnssError.setVisibility(View.GONE);
                trajectoryMapFragment.clearGNSS();
            }
        }

        // WiFi markers
        if (trajectoryMapFragment != null) {
            LatLng wifiLocation = sensorFusion.getLatLngWifiPositioning();
            if (wifiLocation != null && shouldDisplayObservation(
                    wifiLocation, lastDisplayedWifiObservation,
                    now - lastDisplayedWifiTimeMs,
                    WIFI_DISPLAY_MIN_INTERVAL_MS, WIFI_DISPLAY_MIN_DISTANCE_M)) {
                trajectoryMapFragment.updateWifiObservation(wifiLocation);
                lastDisplayedWifiObservation = wifiLocation;
                lastDisplayedWifiTimeMs = now;
            }
        }

        previousPosX = pdrValues[0];
        previousPosY = pdrValues[1];
    }

    private boolean shouldDisplayObservation(LatLng candidate, LatLng previous,
                                             long ageMs, long minIntervalMs,
                                             double minDistanceMeters) {
        if (candidate == null) return false;
        if (previous == null) return true;
        if (ageMs < minIntervalMs) return false;
        return UtilFunctions.distanceBetweenPoints(previous, candidate) >= minDistanceMeters;
    }

    private float smoothHeadingDeg(float newHeadingDeg) {
        if (Float.isNaN(newHeadingDeg) || Float.isInfinite(newHeadingDeg)) {
            return smoothedHeadingDeg != null ? smoothedHeadingDeg : 0f;
        }
        if (smoothedHeadingDeg == null) {
            smoothedHeadingDeg = newHeadingDeg;
            return newHeadingDeg;
        }
        float delta = newHeadingDeg - smoothedHeadingDeg;
        while (delta > 180f) delta -= 360f;
        while (delta < -180f) delta += 360f;
        smoothedHeadingDeg = smoothedHeadingDeg + HEADING_SMOOTHING_ALPHA * delta;
        return smoothedHeadingDeg;
    }

    private void blinkingRecordingIcon() {
        Animation blinking = new AlphaAnimation(1, 0);
        blinking.setDuration(800);
        blinking.setInterpolator(new LinearInterpolator());
        blinking.setRepeatCount(Animation.INFINITE);
        blinking.setRepeatMode(Animation.REVERSE);
        recIcon.startAnimation(blinking);
    }

    @Override
    public void onPause() {
        super.onPause();
        refreshDataHandler.removeCallbacks(refreshDataTask);
    }

    @Override
    public void onResume() {
        super.onResume();
        if (!this.settings.getBoolean("split_trajectory", false)) {
            refreshDataHandler.postDelayed(refreshDataTask, 500);
        }
    }

    private int testPointIndex = 0;

    private static class TestPoint {
        final int index;
        final long timestampMs;
        final double lat;
        final double lng;

        TestPoint(int index, long timestampMs, double lat, double lng) {
            this.index = index;
            this.timestampMs = timestampMs;
            this.lat = lat;
            this.lng = lng;
        }
    }

    private final List<TestPoint> testPoints = new ArrayList<>();
}
