package com.openpositioning.PositionMe.utils;

import android.content.Context;
import android.content.SharedPreferences;

import androidx.annotation.Nullable;

import org.json.JSONObject;

/**
 * IndoorSelectionStore
 * Stores the currently selected indoor venue so it can be used by:
 *  - IndoorMapActivity (floor display) later
 *  - Recording/upload tagging later
 *
 * Kept as a small SharedPreferences wrapper to avoid passing JSON everywhere.
 */
public final class IndoorSelectionStore {

    private static final String PREFS_NAME = "indoor_selection";

    private static final String KEY_SELECTED_VENUE_JSON = "selected_venue_json";
    private static final String KEY_SELECTED_VENUE_ID   = "selected_venue_id";
    private static final String KEY_SELECTED_VENUE_NAME = "selected_venue_name";

    private IndoorSelectionStore() {
        // Utility class
    }

    public static void saveSelectedVenue(Context context,
                                         @Nullable String venueId,
                                         @Nullable String venueName,
                                         @Nullable JSONObject venueJson) {

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);

        prefs.edit()
                .putString(KEY_SELECTED_VENUE_ID, venueId)
                .putString(KEY_SELECTED_VENUE_NAME, venueName)
                .putString(KEY_SELECTED_VENUE_JSON, venueJson == null ? null : venueJson.toString())
                .apply();
    }

    @Nullable
    public static String getSelectedVenueId(Context context) {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .getString(KEY_SELECTED_VENUE_ID, null);
    }

    @Nullable
    public static String getSelectedVenueName(Context context) {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .getString(KEY_SELECTED_VENUE_NAME, null);
    }

    @Nullable
    public static String getSelectedVenueJson(Context context) {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .getString(KEY_SELECTED_VENUE_JSON, null);
    }

    public static void clear(Context context) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .clear()
                .apply();
    }
}
