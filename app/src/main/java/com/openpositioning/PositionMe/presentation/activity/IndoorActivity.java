package com.openpositioning.PositionMe.presentation.activity;
import com.openpositioning.PositionMe.R;
import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.List;

public class IndoorActivity extends AppCompatActivity {

    private WifiManager wifiManager;
    private ListView wifiListView;
    private Button scanButton;

    private ArrayAdapter<String> adapter;
    private List<String> wifiList = new ArrayList<>();

    private static final int PERMISSIONS_REQUEST_CODE = 101;

    // Receiver for Wi-Fi scan completion
    private final BroadcastReceiver wifiScanReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            boolean success = intent.getBooleanExtra(
                    WifiManager.EXTRA_RESULTS_UPDATED, false);

            if (success) {
                displayScanResults();
            } else {
                Toast.makeText(IndoorActivity.this,
                        "Wi-Fi scan failed", Toast.LENGTH_SHORT).show();
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_indoor);

        wifiManager = (WifiManager) getApplicationContext()
                .getSystemService(Context.WIFI_SERVICE);

        wifiListView = findViewById(R.id.wifiListView);
        scanButton = findViewById(R.id.scanButton);

        adapter = new ArrayAdapter<>(
                this,
                android.R.layout.simple_list_item_1,
                wifiList
        );
        wifiListView.setAdapter(adapter);

        scanButton.setOnClickListener(v -> checkAndRequestPermissions());
    }

    // Permission handling (same idea as Lab 2.1 – Location)
    private void checkAndRequestPermissions() {

        if (ContextCompat.checkSelfPermission(
                this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    PERMISSIONS_REQUEST_CODE
            );
        } else {
            startWifiScan();
        }
    }

    private void startWifiScan() {

        if (!wifiManager.isWifiEnabled()) {
            Toast.makeText(this,
                    "Enabling Wi-Fi…", Toast.LENGTH_SHORT).show();
            wifiManager.setWifiEnabled(true);
        }

        // Listen for scan completion
        registerReceiver(
                wifiScanReceiver,
                new IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION)
        );

        try {
            boolean success = wifiManager.startScan();
            if (!success) {
                Toast.makeText(this,
                        "Scan could not start", Toast.LENGTH_SHORT).show();
            }
        } catch (SecurityException e) {
            Toast.makeText(this,
                    "Location permission not granted",
                    Toast.LENGTH_SHORT).show();
        }

    }

    private void displayScanResults() {

        List<ScanResult> results = wifiManager.getScanResults();
        wifiList.clear();

        for (ScanResult result : results) {
            String entry =
                    "SSID: " + result.SSID +
                            "\nRSSI: " + result.level + " dBm";
            wifiList.add(entry);
        }

        adapter.notifyDataSetChanged();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            unregisterReceiver(wifiScanReceiver);
        } catch (IllegalArgumentException e) {
            // Receiver already unregistered – safe to ignore
        }
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode,
            @NonNull String[] permissions,
            @NonNull int[] grantResults) {

        super.onRequestPermissionsResult(
                requestCode, permissions, grantResults);

        if (requestCode == PERMISSIONS_REQUEST_CODE &&
                grantResults.length > 0 &&
                grantResults[0] == PackageManager.PERMISSION_GRANTED) {

            startWifiScan();
        } else {
            Toast.makeText(this,
                    "Location permission required",
                    Toast.LENGTH_SHORT).show();
        }
    }
}
