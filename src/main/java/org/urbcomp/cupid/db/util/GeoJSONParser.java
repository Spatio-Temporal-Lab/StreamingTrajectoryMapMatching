package org.urbcomp.cupid.db.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.CandidatePoint;

import java.io.File;
import java.sql.Timestamp;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

public class GeoJSONParser {
    /**
     * Parses the raw points (latitude, longitude) from the GeoJSON string.
     *
     * @param geoJsonPath Path to the GeoJSON file.
     * @return A list of raw GPS points extracted from the GeoJSON.
     * @throws Exception In case of parsing errors.
     */
    public static List<GPSPoint> parseRawPointsFromGeoJSON(String geoJsonPath) throws Exception {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(geoJsonPath));

        List<GPSPoint> rawPoints = new ArrayList<>();

        // Iterate through the features array
        JsonNode features = root.get("features");
        for (JsonNode feature : features) {
            JsonNode properties = feature.get("properties");

            // Extract rawLat, rawLon, and time from properties
            double rawLat = properties.get("rawLat").asDouble();
            double rawLon = properties.get("rawLon").asDouble();
            String timeString = properties.get("time").asText();
            Timestamp timestamp = parseTimestamp(timeString);

            GPSPoint rawPoint = new GPSPoint(timestamp, rawLon, rawLat);
            rawPoints.add(rawPoint);
        }

        return rawPoints;
    }

    /**
     * Parses the candidate points (latitude, longitude) from the GeoJSON string.
     *
     * @param geoJsonPath Path to the GeoJSON file.
     * @return A list of candidate points extracted from the GeoJSON.
     * @throws Exception In case of parsing errors.
     */
    public static List<CandidatePoint> parseCandidatePointsFromGeoJSON(String geoJsonPath) throws Exception {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(geoJsonPath));

        List<CandidatePoint> candidatePoints = new ArrayList<>();

        // Iterate through the features array
        JsonNode features = root.get("features");
        for (JsonNode feature : features) {
            JsonNode properties = feature.get("properties");
            JsonNode geometry = feature.get("geometry");
            JsonNode coordinates = geometry.get("coordinates");

            // Extract candidate point's longitude and latitude from coordinates
            double errorDistance = properties.get("errorDistanceInMeter").asDouble();
            double offsetInMeter = properties.get("offsetInMeter").asDouble();
            int matchedIndex = properties.get("matchedIndex").asInt();
            int roadSegmentId = properties.get("roadSegmentId").asInt();

            double candidateLon = coordinates.get(0).asDouble();
            double candidateLat = coordinates.get(1).asDouble();

            CandidatePoint candidatePoint = new CandidatePoint(candidateLon, candidateLat, roadSegmentId, matchedIndex, errorDistance, offsetInMeter);
            candidatePoints.add(candidatePoint);
        }

        return candidatePoints;
    }

    /**
     * Converts a time string to a Timestamp object.
     *
     * @param timeString The time string to convert.
     * @return The corresponding Timestamp object.
     * @throws ParseException If the time string is not in the expected format.
     */
    private static Timestamp parseTimestamp(String timeString) throws ParseException {
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S");
        return new Timestamp(dateFormat.parse(timeString).getTime());
    }
}
