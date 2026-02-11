package com.openpositioning.PositionMe.utils;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public final class VenueTagger {

    private VenueTagger() {
        // Utility class
    }

    @NonNull
    public static String buildDataIdentifier(@NonNull Context context, @Nullable String floorKey) {

        String venueId = IndoorSelectionStore.getSelectedVenueId(context);
        String venueName = IndoorSelectionStore.getSelectedVenueName(context);

        if (venueId == null || venueId.trim().isEmpty()) {
            venueId = "unknown";
        }
        if (venueName == null || venueName.trim().isEmpty()) {
            venueName = "unknown";
        }

        // Keep it stable + searchable
        StringBuilder sb = new StringBuilder();
        sb.append("venue_id=").append(venueId.trim());
        sb.append("; venue_name=").append(venueName.trim());

        if (floorKey != null && !floorKey.trim().isEmpty()) {
            sb.append("; floor=").append(floorKey.trim());
        }

        return sb.toString();
    }

    public static boolean hasSelectedVenue(@NonNull Context context) {
        String venueId = IndoorSelectionStore.getSelectedVenueId(context);
        return venueId != null && !venueId.trim().isEmpty();
    }
}
