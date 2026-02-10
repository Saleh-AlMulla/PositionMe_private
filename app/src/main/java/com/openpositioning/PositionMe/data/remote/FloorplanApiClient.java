package com.openpositioning.PositionMe.data.remote;

import androidx.annotation.NonNull;

import com.openpositioning.PositionMe.BuildConfig;

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

public class FloorplanApiClient {

    public interface ResultCallback {
        void onSuccess(@NonNull String rawJson);
        void onError(@NonNull String message, @NonNull Exception e);
    }

    private static final String BASE_URL = "https://openpositioning.org";
    private static final MediaType JSON = MediaType.parse("application/json; charset=utf-8");

    private final OkHttpClient client = new OkHttpClient();

    /**
     * Calls:
     * POST https://openpositioning.org/api/live/floorplan/request/{api_key}/?key={master_key}
     * body: { lat, lon, macs[] }
     */
    public void requestNearbyFloorplans(
            @NonNull String userApiKey,
            double lat,
            double lon,
            @NonNull List<String> macs,
            @NonNull ResultCallback cb
    ) {
        try {
            // Build JSON body exactly as the API expects
            JSONObject body = new JSONObject();
            body.put("lat", lat);
            body.put("lon", lon);

            JSONArray macArr = new JSONArray();
            for (String m : macs) macArr.put(m);
            body.put("macs", macArr);

            // IMPORTANT:
            // - userApiKey is in the PATH
            // - master key is the query param "key"
            String url = BASE_URL
                    + "/api/live/floorplan/request/"
                    + userApiKey
                    + "?key="
                    + BuildConfig.OPENPOSITIONING_MASTER_KEY;

            Request req = new Request.Builder()
                    .url(url)
                    .post(RequestBody.create(body.toString(), JSON))
                    .addHeader("Accept", "application/json")
                    .build();

            client.newCall(req).enqueue(new Callback() {
                @Override
                public void onFailure(@NonNull Call call, @NonNull IOException e) {
                    cb.onError("Floorplan request failed (network)", e);
                }

                @Override
                public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                    String raw = response.body() != null ? response.body().string() : "";
                    if (!response.isSuccessful()) {
                        cb.onError("Floorplan request failed HTTP " + response.code() + ": " + raw,
                                new IOException("HTTP " + response.code()));
                        return;
                    }
                    cb.onSuccess(raw);
                }
            });

        } catch (Exception e) {
            cb.onError("Floorplan request failed (client error)", e);
        }
    }
}
