package org.urbcomp.cupid.db.algorithm.mapmatch.dwrmm;

import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AStarShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.*;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import java.util.*;

import static org.urbcomp.cupid.db.model.point.CandidatePoint.calCandidatePoint;
import static org.urbcomp.cupid.db.util.GeoFunctions.getBearingDifference;
import static org.urbcomp.cupid.db.util.GeoFunctions.getDistanceInM;

// 复现论文“An enhanced weight-based real-time map matching algorithm for complex urban networks”
public class DwrmmMapMatcher {

    protected final RoadNetwork roadNetwork;

    protected final AStarShortestPath aStarShortestPath;

    private TimeStep preTimeStep = null;

    private int count = 0;             // GPS点的计数器
    private double meanDistance = 0.0; // 方向差异标准中的平均距离
    private double sumDistance = 0.0;  // 用于计算方向差异的总距离
    private double standardDeviation = 10.0; // 距离误差高斯分布的标准偏差
    private double mean = 0.0;         // 用于标准偏差的动态更新
    private double m2 = 0.0;           // 累积的方差
    private double meanDistanceDifference = 0.0; // 连通性标准中的平均距离误差
    private double sumDistanceDifference = 0.0;  // 用于计算连通性差异的总距离误差

    public DwrmmMapMatcher(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.aStarShortestPath = new AStarShortestPath(roadNetwork);
    }

    private boolean isInitialized = false; // 标记是否已经初始化
    private static final double K_INIT = 0.1; // 置信度阈值

    // 主匹配方法
    public MapMatchedTrajectory dwrmmMapMatch(Trajectory traj) throws AlgorithmExecuteException {
        List<SequenceState> seq = new ArrayList<>();
        CandidatePoint skipPoint = new CandidatePoint();
        skipPoint.setSkip(true);
        seq.add(new SequenceState(skipPoint, traj.getGPSPointList().get(0)));

        for (int i = 1; i < traj.getGPSPointList().size(); i++) {
            GPSPoint p = traj.getGPSPointList().get(i);
            // 检查是否进入了初始化阶段
            if (!isInitialized) {
                // 初始化阶段的保守延迟匹配
                if (!delayedInitialization(traj.getGPSPointList().get(i - 1), p, seq, i)) {
                    continue; // 如果初始化失败，跳过当前点，等待下一个GPS点
                }
                isInitialized = true; // 初始化成功，进入正常匹配流程
                continue;
            }

            // 正常匹配
            mapMatch(p, seq, i);
        }

        assert traj.getGPSPointList().size() == seq.size();
        List<MapMatchedPoint> mapMatchedPointList = new ArrayList<>(seq.size());
        for (SequenceState ss : seq) {
            CandidatePoint candiPt = null;
            if (ss.getState() != null) {
                candiPt = ss.getState();
            }
            mapMatchedPointList.add(new MapMatchedPoint(ss.getObservation(), candiPt));
        }
        return new MapMatchedTrajectory(traj.getTid(), traj.getOid(), mapMatchedPointList);
    }

    // 初始化阶段的保守延迟匹配
    private boolean delayedInitialization(GPSPoint prevPoint, GPSPoint currentPoint, List<SequenceState> seq, int index) throws AlgorithmExecuteException {
        // 获取两个点的候选路段
        TimeStep prevTimeStep = createTimeStep(prevPoint, index - 1);
        TimeStep currentTimeStep = createTimeStep(currentPoint, index);

        if (prevTimeStep == null || currentTimeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, currentPoint));
            return false; // 如果某个点没有候选路段，返回false
        }

        double maxScore = Double.MIN_VALUE;
        CandidatePoint[] points = null;
        // 遍历候选路段，寻找最佳和次佳匹配
        for (CandidatePoint prevCandidate : prevTimeStep.getCandidates()) {
            // 存储最佳和次佳得分
            double maxConfidence = Double.MIN_VALUE;
            double[] bestScores = null;
            double[] secondBestScores = null;
            double bestTotalScore = Double.MIN_VALUE;
            double secondBestTotalScore = Double.MIN_VALUE;
            CandidatePoint preMatchedPoint = null;
            CandidatePoint matchedPoint = null;
            for (CandidatePoint currentCandidate : currentTimeStep.getCandidates()) {
                // 计算距离得分、方向得分、连通性得分
                double distanceScore = calculateDistanceScore(getDistanceInM(prevPoint, currentPoint));

                double directionDifference = getBearingDifference(currentPoint, prevPoint, currentCandidate, prevCandidate);
                double directionWeight = calculateDirectionWeight(directionDifference);
                double directionScore = directionWeight * calculateDirectionScore(directionDifference);

                double linearDistance = getDistanceInM(currentPoint, prevPoint);
                Path path = aStarShortestPath.findShortestPath(currentCandidate, prevCandidate);
                double pathDistance = path.getLengthInMeter();
                double distanceDifference = Math.abs(linearDistance - pathDistance);
                double connectivityWeight = calculateConnectivityWeight(distanceDifference);
                double connectivityScore =  connectivityWeight * calculateConnectivityScore(distanceDifference);

                // 计算总得分
                double totalScore = distanceScore + directionScore + connectivityScore;

                // 更新最佳和次佳得分
                if (totalScore > bestTotalScore) {
                    preMatchedPoint = prevCandidate;
                    matchedPoint = currentCandidate;
                    // 当前得分比最佳得分高，更新次佳为之前的最佳，最佳为当前得分
                    secondBestTotalScore = bestTotalScore;
                    secondBestScores = bestScores;
                    bestTotalScore = totalScore;
                    bestScores = new double[]{distanceScore, directionScore, connectivityScore, 1.0, directionWeight, connectivityWeight};
                } else if (totalScore != bestTotalScore && totalScore > secondBestTotalScore) {
                    // 当前得分比次佳高，但不如最佳得分，更新次佳得分
                    secondBestTotalScore = totalScore;
                    secondBestScores = new double[]{distanceScore, directionScore, connectivityScore};
                }
            }
            // 计算置信度
            if (bestScores != null && secondBestScores != null) {
                double confidence = calculateConfidence(bestScores, secondBestScores);
                maxConfidence = Math.max(maxConfidence, confidence);
            }

            // 如果置信度大于阈值，选择该匹配结果作为初始化
            if (maxConfidence >= K_INIT) {
                if (bestTotalScore > maxScore) {
                    maxScore = bestTotalScore;
                    points = new CandidatePoint[]{preMatchedPoint, matchedPoint};
                }
            }
        }

        if (points != null) {
            seq.remove(seq.size() - 1);
            seq.add(new SequenceState(points[0], prevPoint));
            seq.add(new SequenceState(points[1], currentPoint));
            currentTimeStep.setMatch(points[1]);
            preTimeStep = currentTimeStep;
            return true;
        }
        else {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, currentPoint));
        }
        return false;
    }

    // 计算置信度
    private double calculateConfidence(double[] bestScores, double[] secondBestScores) {
        double distDiff = Math.abs(bestScores[0] - secondBestScores[0]);
        double directionDiff = Math.abs(bestScores[1] - secondBestScores[1]);
        double connectivityDiff = Math.abs(bestScores[2] - secondBestScores[2]);

        double weightDistance = bestScores[3];  // 距离得分的权重
        double weightDirection = bestScores[4]; // 方向得分的权重
        double weightConnectivity = bestScores[5]; // 连通性得分的权重

        return ((weightDistance * distDiff / (bestScores[0] + secondBestScores[0]))
                + (weightDirection * directionDiff / (bestScores[1] + secondBestScores[1]))
                + (weightConnectivity * connectivityDiff / (bestScores[2] + secondBestScores[2]))) / (weightDistance + weightDirection + weightConnectivity);
    }

    // 地图匹配
    public void mapMatch(GPSPoint observation, List<SequenceState> seq, int index) throws AlgorithmExecuteException {
        TimeStep timeStep = createTimeStep(observation, index);
        double matchedDistance = 0;
        double matchedDistanceDifference = 0;
        double[] bestScores = null;
        double[] secondBestScores = null;
        double bestTotalScore = Double.MIN_VALUE;
        double secondBestTotalScore = Double.MIN_VALUE;
        CandidatePoint matchedPoint = null;
        if (timeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, observation));
            preTimeStep = null;
            return;
        }
        for (CandidatePoint candidatePoint : timeStep.getCandidates()) {
            double totalScore;
            double distanceScore;
            double directionScore = 0;
            double connectivityScore = 0;
            double directionWeight = 1.0;
            double connectivityWeight = 1.0;
            double distanceDifference = 0;

            // 位置得分
            RoadSegment roadSegment = roadNetwork.getRoadSegmentById(
                    candidatePoint.getRoadSegmentId()
            );
            ProjectionPoint projectionPoint = ProjectionPoint.calProjection(observation, roadSegment.getPoints(), 0, roadSegment.getPoints().size() - 1);
            distanceScore = calculateDistanceScore(projectionPoint.getErrorDistanceInMeter());

            if (preTimeStep == null) {
                totalScore = distanceScore;
            }
            else {
                // 方向得分
                GPSPoint preObservation = preTimeStep.getObservation();
                CandidatePoint preMatch = preTimeStep.getMatch();
                double directionDifference = getBearingDifference(observation, preObservation, candidatePoint, preMatch);
                if (directionDifference == 0) {
                    distanceScore = 0;
                }
                else {
                    directionWeight = calculateDirectionWeight(directionDifference);
                    directionScore = directionWeight * calculateDirectionScore(directionDifference);
                }

                // 连通性得分
                double linearDistance = getDistanceInM(observation, preObservation);
                Path path = aStarShortestPath.findShortestPath(preMatch, candidatePoint);
                double pathDistance = path.getLengthInMeter();
                distanceDifference = Math.abs(linearDistance - pathDistance);
                connectivityWeight = calculateConnectivityWeight(distanceDifference);
                connectivityScore =  connectivityWeight * calculateConnectivityScore(distanceDifference);

                totalScore = distanceScore + directionScore + connectivityScore;
//                System.out.println("distanceScore: " + distanceScore);
//                System.out.println("directionWeight: " + directionWeight);
//                System.out.println("directionScore: " + directionScore);
//                System.out.println("connectivityWeight: " + connectivityWeight);
//                System.out.println("connectivityScore: " + connectivityScore);
//                System.out.println("standardDeviation: " + standardDeviation);
//                System.out.println("meanDistance: " + meanDistance);
//                System.out.println("meanDistanceDifference: " + meanDistanceDifference);
            }

            // 更新最佳和次佳得分
            if (totalScore > bestTotalScore) {
                matchedPoint = candidatePoint;
                matchedDistance = projectionPoint.getErrorDistanceInMeter();
                matchedDistanceDifference = distanceDifference;
                // 当前得分比最佳得分高，更新次佳为之前的最佳，最佳为当前得分
                secondBestTotalScore = bestTotalScore;
                secondBestScores = bestScores;
                bestTotalScore = totalScore;
                bestScores = new double[]{distanceScore, directionScore, connectivityScore, 1.0, directionWeight, connectivityWeight};
            } else if (totalScore != bestTotalScore && totalScore > secondBestTotalScore) {
                // 当前得分比次佳高，但不如最佳得分，更新次佳得分
                secondBestTotalScore = totalScore;
                secondBestScores = new double[]{distanceScore, directionScore, connectivityScore};
            }
        }

        //计算置信度
        if (bestScores != null && secondBestScores != null) {
            double confidence = calculateConfidence(bestScores, secondBestScores);
            if (confidence < K_INIT) {
                matchedPoint = calCandidatePoint(observation, roadNetwork.getRoadSegmentById(preTimeStep.getMatch().getRoadSegmentId()));
            }
        }

        // 更新参数
        count++;
        updateStandardDeviation(matchedDistance);
        if (preTimeStep != null) {
            updateMeanDistance(getDistanceInM(observation, preTimeStep.getObservation()));
            updateMeanDistanceDifference(matchedDistanceDifference);
        }

        seq.add(new SequenceState(matchedPoint, observation));
        timeStep.setMatch(matchedPoint);
        preTimeStep = timeStep;
    }

    /**
     * 计算距离标准的得分（高斯分布）
     *
     * @param distance    GPS点到候选路段的垂直距离，单位为米
     * @return 距离标准的得分，值越高表示更可能为正确路段
     */
    public double calculateDistanceScore(double distance) {
        // sigmaG 距离误差的标准偏差，越大表明误差越大
        double sigmaG = getStandardDeviation();
        return (1 / (Math.sqrt(2 * Math.PI) * sigmaG)) * Math.exp(-(Math.pow(distance, 2) / (2 * Math.pow(sigmaG, 2))));
    }

    /**
     * 计算距离标准的权重
     *
     * @param hdop   水平定位精度，值越小表示定位越精确
     * @param HDOP1  HDOP的下限阈值，hdop 小于等于此值时，权重为1
     * @param HDOP2  HDOP的上限阈值，hdop 大于等于此值时，权重为0
     * @return 距离标准的权重，范围在0到1之间
     */
    public double calculateDistanceWeight(double hdop, double HDOP1, double HDOP2) {
        if (hdop <= HDOP1) {
            return 1;
        } else if (hdop < HDOP2) {
            return (hdop - HDOP1) / (HDOP2 - HDOP1);
        } else {
            return 0;
        }
    }

    /**
     * 计算航向差异标准的得分
     *
     * @param thetaJi     当前GPS点的航向与候选路段方向的夹角，单位为度（0到180度）
     * @param deltaTheta  前后GPS点之间的航向差异，单位为度（0到180度）
     * @param step        算法步骤：1表示初始化阶段，2表示追踪匹配阶段
     * @return 航向差异标准的得分，值越高表示更可能为正确路段
     */
    public double calculateHeadingScore(double thetaJi, double deltaTheta, int step) {
        double A = (step == 1) ? 1 : 0; // 初始化步骤时使用
        double B = (step == 2) ? 1 : 0; // 追踪匹配步骤时使用
        return (1 + A * Math.cos(Math.toRadians(thetaJi)) + B * Math.cos(Math.toRadians(deltaTheta))) / 2;
    }

    /**
     * 计算航向差异标准的权重
     *
     * @param velocity     当前GPS点的速度，单位为米/秒
     * @param prevVelocity 前一个GPS点的速度，单位为米/秒
     * @param V1           速度下限阈值，当速度小于V1时，航向权重为0
     * @param V2           速度上限阈值，当速度大于V2时，航向权重为1
     * @return 航向差异标准的权重，范围在0到1之间
     */
    public double calculateHeadingWeight(double velocity, double prevVelocity, double V1, double V2) {
        if (velocity < V1 || prevVelocity < V1) {
            return 0;
        } else if (velocity > V2 && prevVelocity > V2) {
            return 1;
        } else {
            return (velocity + prevVelocity - 2 * V1) / (2 * (V2 - V1));
        }
    }

    /**
     * 计算方向差异标准的得分
     *
     * @param betaJ GPS点连线方向与候选路段方向的夹角，单位为度（0到180度）
     * @return 方向差异标准的得分，值越高表示更可能为正确路段
     */
    public double calculateDirectionScore(double betaJ) {
        double cosValue = Math.cos(Math.toRadians(betaJ));
        return (1 + cosValue) / 2;
    }

    /**
     * 计算方向差异标准的权重
     *
     * @param distance     当前GPS点与前一个GPS点之间的距离，单位为米
     * @return 方向差异标准的权重，范围在0到1之间
     */
    public double calculateDirectionWeight(double distance) {
        if (count < 20) {
            return 1;
        }
        // meanDistance 两个GPS点之间的平均距离，单位为米
        double meanDistance = getMeanDistance();
        if (distance > meanDistance) {
            return 1;
        } else {
            return distance / meanDistance;
        }
    }

    /**
     * 计算路段连通性标准的得分
     *
     * @param distanceDifference 两个GPS点实际行驶距离与投影距离的差值，单位为米
//     * @param alpha  灵敏度调节参数，控制连通性对最终评分的影响，值越大则连通性影响越小
     * @return 路段连通性标准的得分，值越高表示更可能为正确路段
     */
    public double calculateConnectivityScore(double distanceDifference) {
        return Math.exp(-0.4 * distanceDifference);
    }

    /**
     * 计算路段连通性标准的权重
     *
     * @param distanceDifference     当前GPS点的距离差值，单位为米
     * @return 路段连通性标准的权重，范围在0到1之间
     */
    public double calculateConnectivityWeight(double distanceDifference) {
        if (count < 20) {
            return 1.0;
        }
        //distanceDifferenceMean 所有GPS点的平均距离差值，单位为米
        double distanceDifferenceMean = getMeanDistanceDifference();
        if (distanceDifference <= distanceDifferenceMean) {
            return 1.0;
        } else {
            return calculateConnectivityScore(distanceDifference);
        }
    }

    // 创建时间步，初始化候选点
    private TimeStep createTimeStep(GPSPoint pt, int index) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                50.0,
                index
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }

    // 更新距离标准的标准偏差 (Welford算法)
    private void updateStandardDeviation(double newDistance) {
        if (count == 1) {
            return;
        }
        double delta = newDistance - mean;
        mean += delta / count;
        double delta2 = newDistance - mean;
        m2 += delta * delta2;
        standardDeviation = Math.sqrt(m2 / (count - 1));
    }


    // 更新方向差异标准的平均距离 (累加更新)
    private void updateMeanDistance(double newDistance) {
        if (count == 1) {
            return;
        }
        if (newDistance > 50) {
            newDistance = 50;
        }
        sumDistance += newDistance;
        meanDistance = sumDistance / (count - 1);
    }

    // 更新连通性标准的平均误差距离 (累加更新)
    private void updateMeanDistanceDifference(double newDistanceDifference) {
        if (count == 1) {
            return;
        }
        if (newDistanceDifference > 50) {
            newDistanceDifference = 50;
        }
        sumDistanceDifference += newDistanceDifference;
        meanDistanceDifference = sumDistanceDifference / (count - 1);
    }

    // 获取当前的标准偏差
    private double getStandardDeviation() {
        return standardDeviation;
    }

    // 获取当前的平均距离
    private double getMeanDistance() {
        return meanDistance;
    }

    // 获取当前的平均距离误差
    private double getMeanDistanceDifference() {
        return meanDistanceDifference;
    }
}
