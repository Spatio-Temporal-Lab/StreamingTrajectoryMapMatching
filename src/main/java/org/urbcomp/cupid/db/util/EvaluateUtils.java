package org.urbcomp.cupid.db.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.geojson.Point;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class EvaluateUtils {

    static RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

    public static double calculateAccuracy(String baseFile, String matchFile, int sampleRate) {
        List<Double> idAcc = new ArrayList<>();
        try (BufferedReader br1 = new BufferedReader(new FileReader(baseFile));
             BufferedReader br2 = new BufferedReader(new FileReader(matchFile));
        ) {
            String line1, line2;
            int index = 0;
            while ((line1 = br1.readLine()) != null && (line2 = br2.readLine()) != null && index < 1000) {
                index++;
                FeatureCollectionWithProperties fcp1 = new ObjectMapper().readValue(line1, FeatureCollectionWithProperties.class);
                FeatureCollectionWithProperties fcp2 = new ObjectMapper().readValue(line2, FeatureCollectionWithProperties.class);
                double acc = getRoadIDAccuracy(fcp1, fcp2, sampleRate);
                idAcc.add(acc);
                System.out.println("index:" + index + "IDACC: " + acc);
                System.out.println();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("IDACC: " + idAcc.stream().mapToDouble(Double::doubleValue).sum() / idAcc.size());
        return 0;
    }

    /**
     * @param fcp1       真实匹配点的特征集合
     * @param fcp2       地图匹配结果的特征集合
     * @param sampleRate 采样率
     * @return accuracy 准确率
     */
    private static double getRoadIDAccuracy(FeatureCollectionWithProperties fcp1, FeatureCollectionWithProperties fcp2, double sampleRate) {
        List<Feature> features1 = fcp1.getFeatures();
        List<Feature> features2 = fcp2.getFeatures();

        if (features1.isEmpty() || features2.isEmpty()) {
            return 0.0;
        }

        int totalPoints = features1.size();
        List<Feature> sampleFeatures = new ArrayList<>();

        List<Integer> labelList = new ArrayList<>();
        List<Integer> resultList = new ArrayList<>();

        for (int i = 0; i < totalPoints; i += (int) (sampleRate + 1)) {
            Feature feature = features1.get(i);
            sampleFeatures.add(feature);
            labelList.add(feature.getProperty("roadSegmentId"));
        }

        for (Feature feature : features2) {
            resultList.add(feature.getProperty("roadSegmentId"));
        }

        int errorPointsCount = 0;
        List<Integer> mayBeErrorLabel = new ArrayList<>();
        int minSize = Math.min(resultList.size(), labelList.size());

        for (int i = 0; i < minSize; i++) {
            int label = labelList.get(i);
            int result = resultList.get(i);
            if (label == result) {
                if (!mayBeErrorLabel.isEmpty()) {
                    errorPointsCount += checkError(mayBeErrorLabel, label);
                    mayBeErrorLabel.clear();
                }
            } else if (i > 0 && isStartOrEnd(features2.get(i), result)) {
                mayBeErrorLabel.add(result);
            } else if (i > 0 && isStartOrEnd(sampleFeatures.get(i), label)) {
                errorPointsCount += checkError(result, label);
            } else {
                errorPointsCount++;
            }
        }
        return 1 - errorPointsCount * 1.0 / minSize;
    }

    /**
     * 检查一组可能有错误的道路段
     *
     * @param mayBeErrorList 可能错误的路段集合
     * @param labelId        实际路段
     * @return 错误的匹配数
     */
    private static int checkError(List<Integer> mayBeErrorList, int labelId) {
        int errorCount = 0;
        List<RoadNode> checkLabel = getRoadNodesByDirection(labelId);

        for (int result : mayBeErrorList) {
            List<RoadNode> checkResult = getRoadNodesByDirection(result);
            if (isError(checkLabel, checkResult)) {
                System.out.println("label1: " + labelId + " result1 " + result);
                errorCount++;
            }
        }
        return errorCount;
    }

    /**
     * 检查单个可能有错误的道路段
     *
     * @param result  单个道路段
     * @param labelId 实际路段
     * @return 错误的匹配数
     */
    public static int checkError(int result, int labelId) {
        List<RoadNode> checkLabel = getRoadNodesByDirection(labelId);
        List<RoadNode> checkResult = getRoadNodesByDirection(result);

        if (isError(checkLabel, checkResult)) {
            System.out.println("label1: " + labelId + " result1 " + result);
            return 1;
        }
        return 0;
    }

    /**
     * 根据道路段的方向获取结点
     *
     * @param id 路段 id
     * @return 结点集合
     */
    private static List<RoadNode> getRoadNodesByDirection(int id) {
        List<RoadNode> nodes = new ArrayList<>();
        RoadSegment segment = roadNetwork.getRoadSegmentById(id);
        int direction = segment.getDirection().value();

        if (direction == 1) { // 双向道路
            nodes.add(segment.getEndNode());
            nodes.add(segment.getStartNode());
        } else if (direction == 2) { // 正向道路
            nodes.add(segment.getStartNode());
        } else if (direction == 3) { // 反向道路
            nodes.add(segment.getEndNode());
        }
        return nodes;
    }

    /**
     * 检查是否存在错误匹配点
     *
     * @param checkLabel  真实点集合
     * @param checkResult 匹配点集合
     * @return 是否存在错误匹配点
     */
    private static boolean isError(List<RoadNode> checkLabel, List<RoadNode> checkResult) {
        for (RoadNode node1 : checkResult) {
            for (RoadNode node2 : checkLabel) {
                if (node1.getLat() == node2.getLat() && node1.getLng() == node2.getLng()) {
                    return false;
                }
            }
        }
        return true;
    }

    private static int checkError2(int labelId, int resultId, CandidatePoint label, CandidatePoint result) {
        RoadSegment rsResult = roadNetwork.getRoadSegmentById(resultId);
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(labelId);

        boolean isStartEndMatch = (rsResult.getStartNode().getLat() == rsLabel.getEndNode().getLat() &&
                rsResult.getStartNode().getLng() == rsLabel.getEndNode().getLng()) ||
                (rsResult.getEndNode().getLat() == rsLabel.getStartNode().getLat() &&
                        rsResult.getEndNode().getLng() == rsLabel.getStartNode().getLng());

        return isStartEndMatch ? 0 : 1;
    }


    public static int checkError(int result, int labelId, MapMatchedPoint errorLabel, MapMatchedPoint errorResult) {
        int errorCount = 0;
        List<RoadNode> checkResult = new ArrayList<>();
        List<RoadNode> checkLabel = new ArrayList<>();
        if (roadNetwork.getRoadSegmentById(result).getDirection().value() == 1) {
            checkResult.add(roadNetwork.getRoadSegmentById(result).getEndNode());
            checkResult.add(roadNetwork.getRoadSegmentById(result).getStartNode());
        } else if (roadNetwork.getRoadSegmentById(result).getDirection().value() == 2) {
            checkResult.add(roadNetwork.getRoadSegmentById(result).getEndNode());
        } else if (roadNetwork.getRoadSegmentById(result).getDirection().value() == 3) {
            checkResult.add(roadNetwork.getRoadSegmentById(result).getStartNode());
        }

        checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getEndNode());
        checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getStartNode());


        boolean wrong = true;
        for (RoadNode node1 : checkResult) {
            for (RoadNode node2 : checkLabel) {
                if (node1.getLat() == node2.getLat() && node1.getLng() == node2.getLng()) {
                    wrong = false;
                }
            }
        }
        if (wrong) {
//            System.out.println("label1: " + labelId + " result1:" + result );
//            System.out.println("labelPoint:" + errorLabel + " resultPoint:" + errorResult);
//            System.out.println();
            errorCount++;
        }
        return errorCount;
    }


    /**
     * @param p      地图匹配结果的特征
     * @param roadId 匹配结果的路段 id
     * @return isStartOrEnd 匹配点是起始点或者结束点
     */
    private static boolean isStartOrEnd(Feature p, int roadId) {
        Point point = (Point) p.getGeometry();
        double pointLat = point.getCoordinates().getLatitude();
        double pointLng = point.getCoordinates().getLongitude();

        RoadSegment roadSegment = roadNetwork.getRoadSegmentById(Math.abs(roadId));
        RoadNode start = roadSegment.getStartNode();
        RoadNode end = roadSegment.getEndNode();

        return (pointLat == start.getLat() && pointLng == start.getLng()) ||
                (pointLat == end.getLat() && pointLng == end.getLng());
    }

    /**
     * @param p      地图匹配点
     * @param roadId 匹配结果的路段 id
     * @return isStartOrEnd 匹配点是起始点或者结束点
     */
    private static boolean isStartOrEnd(MapMatchedPoint p, int roadId) {
        CandidatePoint point = p.getCandidatePoint();
        if (point == null) return false;
        double pointLat = point.getLat();
        double pointLng = point.getLng();
        RoadNode start = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getStartNode();
        RoadNode end = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getEndNode();
        return (pointLat == start.getLat() && pointLng == start.getLng()) ||
                (pointLat == end.getLat() && pointLng == end.getLng());
    }

    private static boolean checkLabel(int label, int result) {
        if (Math.abs(label) == Math.abs(result)) {
            return true;
        }
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(label);
        RoadSegment rsResult = roadNetwork.getRoadSegmentById(result);
        return rsLabel.getStartNode().getLng() == rsResult.getEndNode().getLng() && rsLabel.getStartNode().getLat() == rsResult.getEndNode().getLat() && rsLabel.getLengthInMeter() == rsResult.getLengthInMeter();
    }

    public static double getAccuracy(MapMatchedTrajectory labels, MapMatchedTrajectory results, double sampleRate) {
        List<MapMatchedPoint> labelPoints = labels.getMmPtList();
        List<MapMatchedPoint> resultPoints = results.getMmPtList();
        double totalPoints = labels.getMmPtList().size();
        double samplePoints = results.getMmPtList().size();

        if (totalPoints == 0) {
            return 0.0;
        }

        List<Integer> labelList = new ArrayList<>();    // 存储真实路段 id
        List<MapMatchedPoint> sampleLabelList = new ArrayList<>();  // 存储匹配点
        List<Integer> resultList = new ArrayList<>();   // 存储匹配路段 id

        int step = (int) (sampleRate + 1);
        for (int i = 0; i < totalPoints; i += step) {
            MapMatchedPoint point = labelPoints.get(i);
            labelList.add(point.getCandidatePoint() == null ? 0 : point.getCandidatePoint().getRoadSegmentId());
            sampleLabelList.add(point);
        }

        for (MapMatchedPoint point : resultPoints) {
            resultList.add(point.getCandidatePoint() == null ? 0 : point.getCandidatePoint().getRoadSegmentId());
        }

        int errorPointsCount = 0;
        int minSize = Math.min(resultList.size(), labelList.size());

        for (int i = 0; i < minSize; i++) {
            int label = labelList.get(i);   // 真实路段 id
            int result = resultList.get(i); // 匹配路段 id
            if (checkLabel(label, result)) {

            } else if (i > 0 && result != 0 && label != 0 && (isStartOrEnd(resultPoints.get(i), result) || isStartOrEnd(sampleLabelList.get(i), label))) {
                errorPointsCount += checkError2(label, result, sampleLabelList.get(i).getCandidatePoint(), resultPoints.get(i).getCandidatePoint());
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
