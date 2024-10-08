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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class EvaluateUtils {
    static int errorNum = 0;
    static int totalCorrectNum = 0;
    static int totalNum = 0;
    static double currAcc = 0.0;
    static double totalAcc = 0.0;

    static RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

    public static double calculateAccuracy(String baseFile, String matchFile, int sampleRate) {
        List<Double> idAcc = new ArrayList<>();
        try (BufferedReader br1 = new BufferedReader(new FileReader(baseFile));
             BufferedReader br2 = new BufferedReader(new FileReader(matchFile))) {
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

    private static double getRoadIDAccuracy(FeatureCollectionWithProperties fcp1, FeatureCollectionWithProperties fcp2, double sampleRate) {
        List<Feature> features1 = fcp1.getFeatures();
        List<Feature> features2 = fcp2.getFeatures();
        List<Feature> sampleFeatures1 = new ArrayList<>();
        double totalPoints = features1.size();
        double samplePoints = features2.size();
        if (totalPoints == 0) {
            return 0.0;
        }
        List<Integer> labelList = new ArrayList<>();
        List<Integer> resultList = new ArrayList<>();

        int skipNum = 0;
        boolean flag = true;
        for (int i = 0; i < totalPoints; i++) {
            if (flag) {
                int id1 = features1.get(i).getProperty("roadSegmentId");
                sampleFeatures1.add(features1.get(i));
                labelList.add(id1);
                flag = false;
                if (skipNum == sampleRate) {
                    flag = true;
                }
            } else {
                skipNum++;
                if (skipNum == sampleRate) {
                    skipNum = 0;
                    flag = true;
                }
            }
        }
        for (int i = 0; i < samplePoints; i++) {
            int id2 = features2.get(i).getProperty("roadSegmentId");
            resultList.add(id2);
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
            } else if (i > 0 && (isStartOrEnd(features2.get(i), result))) {
                mayBeErrorLabel.add(result);
            } else if (i > 0 && isStartOrEnd(sampleFeatures1.get(i), label)) {
                errorPointsCount += checkError(result, label);
            } else {
                errorPointsCount++;
            }
        }
        return 1 - errorPointsCount * 1.0 / minSize;
    }

    private static int checkError(List<Integer> mayBeEorrorList, int labelId) {
        int errorCount = 0;
        List<RoadNode> checkLabel = new ArrayList<>();
        if (roadNetwork.getRoadSegmentById(labelId).getDirection().value() == 1) {
            checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getEndNode());
            checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getStartNode());
        } else if (roadNetwork.getRoadSegmentById(labelId).getDirection().value() == 2) {
            checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getStartNode());
        } else if (roadNetwork.getRoadSegmentById(labelId).getDirection().value() == 3) {
            checkLabel.add(roadNetwork.getRoadSegmentById(labelId).getEndNode());
        }
        for (int result : mayBeEorrorList) {
            List<RoadNode> checkResult = new ArrayList<>();
            checkResult.add(roadNetwork.getRoadSegmentById(result).getEndNode());
            checkResult.add(roadNetwork.getRoadSegmentById(result).getStartNode());

            boolean wrong = true;
            for (RoadNode node1 : checkResult) {
                for (RoadNode node2 : checkLabel) {
                    if (node1.getLat() == node2.getLat() && node1.getLng() == node2.getLng()) {
                        wrong = false;
                    }
                }
            }
            if (wrong) {
                System.out.println("label1: " + labelId + " result1 " + result);
                errorCount++;
            }
        }
        return errorCount;
    }


    public static int checkError(int result, int labelId) {
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
            System.out.println("label1: " + labelId + " result1 " + result);
            errorCount++;
        }
        return errorCount;
    }

    private static int checkError2(int labelId, int resultId, CandidatePoint label, CandidatePoint result) {
        int errorCount = 0;

        RoadSegment rsResult = roadNetwork.getRoadSegmentById(resultId);
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(labelId);
        if ((rsResult.getStartNode().getLat() == rsLabel.getEndNode().getLat() && rsResult.getStartNode().getLng() == rsLabel.getEndNode().getLng()) ||
                (rsResult.getEndNode().getLat() == rsLabel.getStartNode().getLat() && rsResult.getEndNode().getLng() == rsLabel.getStartNode().getLng())) {
            return errorCount;
        } else {
            errorCount++;
        }
        return errorCount;
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


    private static boolean isStartOrEnd(Feature p, int roadId) {
        Point point = (Point) p.getGeometry();
        RoadNode start = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getStartNode();
        RoadNode end = roadNetwork.getRoadSegmentById(Math.abs(roadId)).getEndNode();
        if (point.getCoordinates().getLatitude() == start.getLat() && point.getCoordinates().getLongitude() == start.getLng()) {
            return true;
        } else
            return point.getCoordinates().getLatitude() == end.getLat() && point.getCoordinates().getLongitude() == end.getLng();
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

    private static boolean checkLabel(int label, int result, CandidatePoint labelPoint, CandidatePoint resultPoint) {
        if (Math.abs(label) == Math.abs(result)) {
            return true;
        } else if (labelPoint.getLat() == resultPoint.getLat() && labelPoint.getLng() == resultPoint.getLng()) {
            return true;
        }
        RoadSegment rsLabel = roadNetwork.getRoadSegmentById(label);
        RoadSegment rsResult = roadNetwork.getRoadSegmentById(result);
        return rsLabel.getStartNode().getLng() == rsResult.getEndNode().getLng() &&
                rsLabel.getStartNode().getLat() == rsResult.getEndNode().getLat() &&
                rsLabel.getLengthInMeter() == rsResult.getLengthInMeter();
    }

    public static void getAccuracy(MapMatchedTrajectory labels, MapMatchedTrajectory results, double sampleRate) {
        int totalPoints = labels.getMmPtList().size();
        int resultPoints = results.getMmPtList().size();

        // 确保 labels 和 results 的轨迹点数量一致
        if (totalPoints != resultPoints) {
            throw new IllegalArgumentException("Labels and results must have the same number of points.");
        }

        if (totalPoints == 0) {
            currAcc = 0.0;
            return;
        }

        List<Integer> labelList = new ArrayList<>();
        List<MapMatchedPoint> sampleLabelList = new ArrayList<>();
        List<Integer> resultList = new ArrayList<>();

        int skipNum = 0;
        boolean flag = true;

        for (int i = 0; i < totalPoints; i++) {
            CandidatePoint resultPoint = results.getMmPtList().get(i).getCandidatePoint();

            // 检查当前结果点是否需要跳过
            if (resultPoint != null && resultPoint.isSkip()) {
                continue;
            }

            if (flag) {
                // 添加标签点到 labelList
                CandidatePoint labelCp = labels.getMmPtList().get(i).getCandidatePoint();
                if (labelCp == null) {
                    labelList.add(0);
                } else {
                    int labelId = labelCp.getRoadSegmentId();
                    labelList.add(labelId);
                }

                // 添加对应的标签 MapMatchedPoint 到 sampleLabelList
                sampleLabelList.add(labels.getMmPtList().get(i));

                // 添加结果点到 resultList
                if (resultPoint == null) {
                    resultList.add(0);
                } else {
                    int resultId = resultPoint.getRoadSegmentId();
                    resultList.add(resultId);
                }

                // 根据 sampleRate 决定是否继续采样
                flag = skipNum == sampleRate;
            } else {
                skipNum++;
                if (skipNum == sampleRate) {
                    skipNum = 0;
                    flag = true;
                }
            }
        }

        int errorPointsCount = 0;
        int totalComparedPoints = labelList.size(); // 实际参与比较的点数

        // 确定比较的最小长度
        int minSize = Math.min(resultList.size(), labelList.size());

        for (int i = 0; i < minSize; i++) {
            int label = labelList.get(i);
            int result = resultList.get(i);
            CandidatePoint labelCp = sampleLabelList.get(i).getCandidatePoint();
            CandidatePoint resultCp = results.getMmPtList().get(i).getCandidatePoint();

            // 检查标签点和结果点是否匹配
            if (checkLabel(label, result, labelCp, resultCp)) {
                // 匹配正确，不计入错误点数
            } else if (i > 0 && result != 0 && label != 0 &&
                    (isStartOrEnd(results.getMmPtList().get(i), result) || isStartOrEnd(sampleLabelList.get(i), label))) {
                // 处理特殊错误情况
                int count = checkError2(label, result, labelCp, resultCp);
                errorPointsCount += count;
                if (count != 0) {
                    // 可以启用调试输出
                    // System.out.println("index: " + i + " " + resultCp + " result: " + resultCp.getCoordinate() + " label: " + labelCp.getCoordinate());
                }
            } else {
                // 普通错误，增加错误点数
                errorPointsCount++;
                // 可以启用调试输出
                // System.out.println("index: " + i + " " + resultCp + " result: " + resultCp.getCoordinate() + " label: " + labelCp.getCoordinate());
            }
        }

        System.out.println("wrong Points : " + errorPointsCount);

        // 处理所有点都被跳过的情况，避免除以零
        if (totalComparedPoints == 0) {
            currAcc = 0.0;
            return;
        }

        errorNum = errorPointsCount;
        totalCorrectNum += totalComparedPoints - errorPointsCount;
        totalNum += totalComparedPoints;
        currAcc = 1 - (errorPointsCount * 1.0 / totalComparedPoints);
        totalAcc = totalCorrectNum * 1.0 / totalNum;
    }

    public static double getCurrAcc() {
        return currAcc;
    }

    public static double getTotalAcc() {
        return totalAcc;
    }

    public static int getErrorNum() {
        return errorNum;
    }

    public static int getTotalCorrectNum() {
        return totalCorrectNum;
    }

    public static int getTotalNum() {
        return totalNum;
    }
}
