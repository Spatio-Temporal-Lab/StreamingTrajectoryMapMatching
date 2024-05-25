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
            int index = 0;
            while ((line1 = br1.readLine()) != null && (line2 = br2.readLine()) != null) {
                index++;
                List<Point> pointList = new ArrayList<>();
                List<Point> pointList2 = new ArrayList<>();
                FeatureCollectionWithProperties fcp1 = new ObjectMapper().readValue(line1, FeatureCollectionWithProperties.class);
                FeatureCollectionWithProperties fcp2 = new ObjectMapper().readValue(line2, FeatureCollectionWithProperties.class);
                double accuracy = getAccuracy(fcp1, fcp2, pointList, pointList2);
                System.out.println("index:" + index + " accuracy:" + accuracy );
                for (int i = 0; i < pointList.size(); i++ ){
                    System.out.print("pointListLabels:" + pointList.get(i) + " pointListStream:" +pointList2.get(i) + "~~~~~~~~~");
                }
                System.out.println();
                accuracies.add(accuracy);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        double totalAccuracy = accuracies.stream().mapToDouble(Double::doubleValue).sum();
        return totalAccuracy / accuracies.size();
    }

    private static double getAccuracy(FeatureCollectionWithProperties fcp1, FeatureCollectionWithProperties fcp2, List<Point> pointList, List<Point> pointList2) {
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
            int id1 = features1.get(i).getProperty("roadSegmentId");
            Point point2 = (Point) features2.get(i).getGeometry();
            int id2 = features2.get(i).getProperty("roadSegmentId");
            if (point1.getCoordinates().equals(point2.getCoordinates())) {
                matchedPoints++;
            }else if (id1 == id2){
                matchedPoints++;
            }
            else {
                pointList.add(point1);
                pointList2.add(point2);
            }
        }
        return matchedPoints / totalPoints * 100;
    }
}
