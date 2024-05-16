package org.urbcomp.cupid.db.util;

import org.geojson.Feature;
import org.geojson.Point;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class EvaluateUtils {
    public static double calculateAccuracy(String baseFile, String matchFile) {
        List<Double> accuracies = new ArrayList<>();
        try (BufferedReader br1 = new BufferedReader(new FileReader(baseFile));
             BufferedReader br2 = new BufferedReader(new FileReader(matchFile))) {
            String line1, line2;
            int i = 0;
            while ((line1 = br1.readLine()) != null && (line2 = br2.readLine()) != null) {
                FeatureCollectionWithProperties fcp1 = new ObjectMapper().readValue(line1, FeatureCollectionWithProperties.class);
                FeatureCollectionWithProperties fcp2 = new ObjectMapper().readValue(line2, FeatureCollectionWithProperties.class);
                double accuracy = getAccuracy(fcp1, fcp2);
                accuracies.add(accuracy);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        double totalAccuracy = accuracies.stream().mapToDouble(Double::doubleValue).sum();
        double averageAccuracy = totalAccuracy / accuracies.size();
        return averageAccuracy;
    }

    private static double getAccuracy(FeatureCollectionWithProperties fcp1, FeatureCollectionWithProperties fcp2) {
        List<Feature> features1 = fcp1.getFeatures();
        List<Feature> features2 = fcp2.getFeatures();
        assert features1.size() == features2.size();
        double totalPoints = Math.min(features1.size(), features2.size());
        if (totalPoints == 0) {
            return 0.0;
        }
        double matchedPoints = 0;
        for (int i = 0; i < totalPoints; i++) {
            Point point1 = (Point) features1.get(i).getGeometry();
            Point point2 = (Point) features2.get(i).getGeometry();

            if (point1.getCoordinates().equals(point2.getCoordinates())) {
                matchedPoints++;
            }
        }
        return matchedPoints / totalPoints * 100;
    }
}
