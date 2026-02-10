package com.openpositioning.PositionMe.data.remote;

import androidx.annotation.NonNull;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;
import java.util.List;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * FloorplanApiClient
 * Calls: POST /api/live/floorplan/request/{api_key}
 * Body: { lat, lon, macs[] }
 */
public final class FloorplanApiClient {

    public interface ResultCallback {
        void onSuccess(@NonNull String rawJson);
        void onError(@NonNull String message, @NonNull Exception e);
    }

    private static final MediaType JSON
            = MediaType.parse("application/json; charset=utf-8");

    private final OkHttpClient http = new OkHttpClient();

    public void requestNearbyFloorplans(
            @NonNull String apiKey,
            double lat,
            double lon,
            @NonNull List<String> macs,
            @NonNull ResultCallback cb
    ) {
        try {
            String url = "https://openpositioning.org/api/live/floorplan/request/" + apiKey;

            JSONObject payload = new JSONObject();
            payload.put("lat", lat);
            payload.put("lon", lon);

            JSONArray arr = new JSONArray();
            for (String mac : macs) arr.put(mac);
            payload.put("macs", arr);

            RequestBody body = RequestBody.create(payload.toString(), JSON);

            Request req = new Request.Builder()
                    .url(url)
                    .post(body)
                    .addHeader("Accept", "application/json")
                    .build();

            http.newCall(req).enqueue(new Callback() {
                @Override
                public void onFailure(@NonNull Call call, @NonNull IOException e) {
                    cb.onError("Floorplan request failed (network).", e);
                }

                @Override
                public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                    String raw = response.body() != null ? response.body().string() : "";
                    if (!response.isSuccessful()) {
                        cb.onError("Floorplan request failed (HTTP " + response.code() + ").",
                                new IOException(raw));
                        return;
                    }
                    cb.onSuccess(raw);
                }
            });

        } catch (Exception e) {
            cb.onError("Floorplan request failed (payload/build).", e);
        }
    }
}
