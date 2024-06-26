package org.urbcomp.cupid.db.util;

import org.geojson.Feature;
import org.geojson.Point;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class EvaluateUtils {
    public static double calculateAccuracy(String baseFile, String matchFile, int n) {
        List<Double> accuracies = new ArrayList<>();
        int notValid = 0;
        try (BufferedReader br1 = new BufferedReader(new FileReader(baseFile));
             BufferedReader br2 = new BufferedReader(new FileReader(matchFile))) {
            String line1, line2;
            int index = 0;
            while (n > 0) {
                line1 = br1.readLine();
                line2 = br2.readLine();
                if (line1 == null || line2 == null) break;
                index++;
                List<Point> pointList = new ArrayList<>();
                List<Point> pointList2 = new ArrayList<>();
                FeatureCollectionWithProperties fcp1 = new ObjectMapper().readValue(line1, FeatureCollectionWithProperties.class);
                FeatureCollectionWithProperties fcp2 = new ObjectMapper().readValue(line2, FeatureCollectionWithProperties.class);
                double accuracy = getAccuracy(fcp1, fcp2, pointList, pointList2);
                System.out.println("index:" + index + " accuracy:" + accuracy );
//                for (int i = 0; i < pointList.size(); i++ ){
//                    System.out.print("pointListLabels:" + pointList.get(i) + " pointListStream:" +pointList2.get(i) + "~~~~~~~~~");
//                }
                System.out.println();
                if (fcp1.getFeatures().size() == fcp2.getFeatures().size()) accuracies.add(accuracy);
                else notValid++;
                n--;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        double totalAccuracy = accuracies.stream().mapToDouble(Double::doubleValue).sum();
        System.out.println("The number of not valid is: " + notValid);
        writeAccuracyToCSV(accuracies, "accuracy_result.csv");
        return totalAccuracy / accuracies.size();
    }
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
        System.out.println(features1.size() + ":" + features2.size());
        if (features1.size() != features2.size()) {
            System.out.println("doesn't match, can't compare!");
            return -1.0;
        }
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

    public static void writeAccuracyToCSV(List<Double> accuries, String filename) {
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write("index,accuracy\n");

            for (int i = 0; i < accuries.size(); i++) {
                double accuracy = accuries.get(i);
                writer.write((i+1) + "," + accuracy + "\n");
            }

            System.out.println("CSV file created successfully.");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
