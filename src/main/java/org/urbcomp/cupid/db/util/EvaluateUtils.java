package org.urbcomp.cupid.db.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.geojson.Point;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadGraph;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class EvaluateUtils {

    static RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
    public static double calculateAccuracy(String baseFile, String matchFile) {
        List<Double> accuracies = new ArrayList<>();
        List<Double> idAcc =new ArrayList<>();
        double totalPoints = 0;
        double totalMatchedPoint = 0;
        try (BufferedReader br1 = new BufferedReader(new FileReader(baseFile));
             BufferedReader br2 = new BufferedReader(new FileReader(matchFile))) {
            String line1, line2;
            int index = 0;

            while ((line1 = br1.readLine()) != null && (line2 = br2.readLine()) != null && index < 1000) {
                index++;
                List<Point> pointList = new ArrayList<>();
                List<Point> pointList2 = new ArrayList<>();
                FeatureCollectionWithProperties fcp1 = new ObjectMapper().readValue(line1, FeatureCollectionWithProperties.class);
                FeatureCollectionWithProperties fcp2 = new ObjectMapper().readValue(line2, FeatureCollectionWithProperties.class);
                double accuracy = getAccuracy(fcp1, fcp2, pointList, pointList2);
                double acc = getRoadIDAccuracy(fcp1,fcp2);
                idAcc.add(acc);
                totalPoints += fcp1.getFeatures().size();
                totalMatchedPoint += fcp1.getFeatures().size() * accuracy;
                System.out.println("index:" + index + " accuracy:" + accuracy + "IDACC: " + acc );
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
        System.out.println("totalAccuracies: " + totalMatchedPoint/totalPoints);
        System.out.println("IDACC: " + idAcc.stream().mapToDouble(Double::doubleValue).sum()/idAcc.size());
        return totalAccuracy / accuracies.size();
    }

    private static double getRoadIDAccuracy(FeatureCollectionWithProperties fcp1, FeatureCollectionWithProperties fcp2){
        List<Feature> features1 = fcp1.getFeatures();
        List<Feature> features2 = fcp2.getFeatures();
        assert features1.size() == features2.size();
        double totalPoints = Math.min(features1.size(), features2.size());
        if (totalPoints == 0) {
            return 0.0;
        }
        List<Integer> labelList = new ArrayList<>();
        List<Integer> resultList = new ArrayList<>();

        for (int i = 0; i < totalPoints; i++) {
            int id1 = features1.get(i).getProperty("roadSegmentId");
            int id2 = features2.get(i).getProperty("roadSegmentId");
            labelList.add(id1);
            resultList.add(id2);
        }

        int errorPointsCount = 0;
        int mayBeError = 0;
        for (int i = 0; i < labelList.size(); i++){
            int label = labelList.get(i);
            int result = resultList.get(i);
            if (label == result){
                if (mayBeError >3){
                    errorPointsCount += mayBeError;
                    mayBeError =0;
                }
                if (mayBeError > 0){
                    mayBeError = 0;
                }
            }else if (i >0 && result == resultList.get(i-1)){
                mayBeError ++;
            }else {
                errorPointsCount ++;
                if (mayBeError > 0){
                    errorPointsCount += mayBeError;
                    mayBeError = 0;
                }
            }
        }
        return  1 - errorPointsCount * 1.0/ features1.size();
    }

    public static int getDifferentElementCount(Set<Integer> set1, Set<Integer> set2) {
        // 创建副本集合
        Set<Integer> set1Copy = new HashSet<>(set1);
        Set<Integer> set2Copy = new HashSet<>(set2);

        // 移除两个集合的交集部分
        set2Copy.removeAll(set1);
        System.out.println(set2Copy);
        // 计算不同元素的数量
        return  set2Copy.size();
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
            boolean flag1 = true;
            boolean flag2 = true;
            Point point1 = (Point) features1.get(i).getGeometry();
            int id1 = features1.get(i).getProperty("roadSegmentId");
            if (id1 < 0){
                id1 = -id1;
                flag1 = false;
            }
            double endX;
            double endY;
            Point point2 = (Point) features2.get(i).getGeometry();
            int id2 = features2.get(i).getProperty("roadSegmentId");
            if (id2 < 0){
                id2 = -id2;
                flag2 = false;
            }

            double endX0;
            double endY0;
            if (id1 < 0){
                id1 = -id1;
                flag1 = false;
            }

            if (!flag1){
                endX0 = roadNetwork.getRoadSegmentById(id1).getStartNode().getLat();
                endY0 = roadNetwork.getRoadSegmentById(id1).getStartNode().getLng();
            }else {
                endX0 = roadNetwork.getRoadSegmentById(id1).getEndNode().getLat();
                endY0 = roadNetwork.getRoadSegmentById(id1).getEndNode().getLng();
            }


            if (!flag2){
                endX = roadNetwork.getRoadSegmentById(id2).getStartNode().getLat();
                endY = roadNetwork.getRoadSegmentById(id2).getStartNode().getLng();
            }else {
                endX = roadNetwork.getRoadSegmentById(id2).getEndNode().getLat();
                endY = roadNetwork.getRoadSegmentById(id2).getEndNode().getLng();
            }

            if (point1.getCoordinates().equals(point2.getCoordinates())) {
                matchedPoints++;
            }else if (id1 == id2){
                matchedPoints++;
            }else if ( endX== point2.getCoordinates().getLatitude() && endY == point2.getCoordinates().getLongitude() &&
                    RoadGraph.areEdgesAdjacent(roadNetwork.getRoadSegmentById(id2), roadNetwork.getRoadSegmentById(id1))){
                matchedPoints++;
            }else if ( endX0== point1.getCoordinates().getLatitude() && endY0 == point1.getCoordinates().getLongitude() &&
                    RoadGraph.areEdgesAdjacent(roadNetwork.getRoadSegmentById(id1), roadNetwork.getRoadSegmentById(id1))){
                matchedPoints++;
            }
            else {
                pointList.add(point1);
                pointList2.add(point2);
            }
        }
        return matchedPoints / totalPoints * 100;
    }

    public static double getAccuracy(MapMatchedTrajectory label, MapMatchedTrajectory result) {
        List<MapMatchedPoint> features1 = label.getMmPtList();
        List<MapMatchedPoint> features2 = result.getMmPtList();
        assert features1.size() == features2.size();
        double totalPoints = Math.min(features1.size(), features2.size());
        if (totalPoints == 0) {
            return 0.0;
        }
        double matchedPoints = 0;
        for (int i = 0; i < totalPoints; i++) {
            boolean flag1 = true;
            boolean flag2 = true;
            CandidatePoint point1 = features1.get(i).getCandidatePoint();
            int id1 = point1.getRoadSegmentId();
            if (id1 < 0){
                id1 = -id1;
                flag1 = false;
            }
            double endX;
            double endY;
            CandidatePoint point2 = features2.get(i).getCandidatePoint();
            int id2 = point2.getRoadSegmentId();
            if (id2 < 0){
                id2 = -id2;
                flag2 = false;
            }

            double endX0;
            double endY0;
            if (id1 < 0){
                id1 = -id1;
                flag1 = false;
            }

            if (!flag1){
                endX0 = roadNetwork.getRoadSegmentById(id1).getStartNode().getLat();
                endY0 = roadNetwork.getRoadSegmentById(id1).getStartNode().getLng();
            }else {
                endX0 = roadNetwork.getRoadSegmentById(id1).getEndNode().getLat();
                endY0 = roadNetwork.getRoadSegmentById(id1).getEndNode().getLng();
            }


            if (!flag2){
                endX = roadNetwork.getRoadSegmentById(id2).getStartNode().getLat();
                endY = roadNetwork.getRoadSegmentById(id2).getStartNode().getLng();
            }else {
                endX = roadNetwork.getRoadSegmentById(id2).getEndNode().getLat();
                endY = roadNetwork.getRoadSegmentById(id2).getEndNode().getLng();
            }

            if (point1.getCoordinates().equals(point2.getCoordinates())) {
                matchedPoints++;
            }else if (id1 == id2){
                matchedPoints++;
            }else if ( endX== point2.getLat() && endY == point2.getLng() &&
                    RoadGraph.areEdgesAdjacent(roadNetwork.getRoadSegmentById(id2), roadNetwork.getRoadSegmentById(id1))){
                matchedPoints++;
            }else if ( endX0== point1.getLat() && endY0 == point1.getLng() &&
                    RoadGraph.areEdgesAdjacent(roadNetwork.getRoadSegmentById(id1), roadNetwork.getRoadSegmentById(id1))){
                matchedPoints++;
            }
        }
        return matchedPoints / totalPoints * 100;
    }


}
