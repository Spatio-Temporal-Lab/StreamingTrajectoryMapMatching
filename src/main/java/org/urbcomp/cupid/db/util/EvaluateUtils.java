package org.urbcomp.cupid.db.util;

import org.geojson.Feature;
import org.geojson.Point;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class EvaluateUtils {
    static RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

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

    private static boolean isStartOrEnd(MapMatchedPoint p, int roadId) {
        CandidatePoint point = p.getCandidatePoint();
        RoadNode start = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getStartNode();
        RoadNode end = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getEndNode();
        if (point == null) {
            return false;
        }
        if (point.getLat() == start.getLat() && point.getLng() == start.getLng()) {
            return true;
        } else return point.getLat() == end.getLat() && point.getLng() == end.getLng();
    }

    private static boolean checkLabel(int label, int result) {
        if (Math.abs(label) == Math.abs(result)) {
            return true;
        }
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(label);
        RoadSegment rsResult = roadNetwork.getRoadSegmentById(result);
        return rsLabel.getStartNode().getLng() == rsResult.getEndNode().getLng() && rsLabel.getStartNode().getLat() == rsResult.getEndNode().getLat() && rsLabel.getLengthInMeter() == rsResult.getLengthInMeter();
    }

    private static int checkError2(int labelId, int resultId, CandidatePoint label, CandidatePoint result) {
        int errorCount = 0;

        RoadSegment rsResult = roadNetwork.getRoadSegmentById(resultId);
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(labelId);
        if ((rsResult.getStartNode().getLat() == rsLabel.getEndNode().getLat() && rsResult.getStartNode().getLng() == rsLabel.getEndNode().getLng()) ||
                (rsResult.getEndNode().getLat() == rsLabel.getStartNode().getLat() && rsResult.getEndNode().getLng() == rsLabel.getStartNode().getLng())) {
        } else {
            errorCount++;
        }
        return errorCount;
    }

    public static double getAccuracy(MapMatchedTrajectory labels, MapMatchedTrajectory results, double sampleRate) {
        double totalPoints = labels.getMmPtList().size();
        double samplePoints = results.getMmPtList().size();
        if (totalPoints == 0) {
            return 0.0;
        }
        List<Integer> labelList = new ArrayList<>();
        List<MapMatchedPoint> sampleLabelList = new ArrayList<>();
        List<Integer> resultList = new ArrayList<>();

        int skipNum = 0;
        boolean flag = true;
        for (int i = 0; i < totalPoints; i++) {
            if (flag) {
                if (labels.getMmPtList().get(i).getCandidatePoint() == null) {
                    labelList.add(0);
                } else {
                    int id1 = labels.getMmPtList().get(i).getCandidatePoint().getRoadSegmentId();
                    labelList.add(id1);
                }
                sampleLabelList.add(labels.getMmPtList().get(i));
                flag = skipNum == sampleRate;
            } else {
                skipNum++;
                if (skipNum == sampleRate) {
                    skipNum = 0;
                    flag = true;
                }
            }
        }
        for (int i = 0; i < samplePoints; i++) {
            if (results.getMmPtList().get(i).getCandidatePoint() == null) {
                resultList.add(0);
            } else {
                int id2 = results.getMmPtList().get(i).getCandidatePoint().getRoadSegmentId();
                resultList.add(id2);
            }
        }

        int errorPointsCount = 0;
        int minSize = Math.min(resultList.size(), labelList.size());
        for (int i = 0; i < minSize; i++) {
            int label = labelList.get(i);
            int result = resultList.get(i);
            if (checkLabel(label, result)) {

            } else if (i > 0 && result != 0 && label != 0 && ((isStartOrEnd(results.getMmPtList().get(i), result)) || isStartOrEnd(sampleLabelList.get(i), label))) {
                int count = checkError2(label, result, sampleLabelList.get(i).getCandidatePoint(), results.getMmPtList().get(i).getCandidatePoint());
                errorPointsCount += count;
            } else {
//                System.out.println("label3: " + label + " result3:" + result );
//                System.out.println("labelPoint:" + sampleLabelList.get(i) + " resultPoint:" + results.getMmPtList().get(i));
//                System.out.println();
                errorPointsCount++;
            }
        }
        System.out.println("wrong Points : " + errorPointsCount);
        return 1 - errorPointsCount * 1.0 / minSize;
    }

}
