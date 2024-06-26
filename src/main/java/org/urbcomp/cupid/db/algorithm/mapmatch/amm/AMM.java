package org.urbcomp.cupid.db.algorithm.mapmatch.amm;

import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.CandidateAttributes;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.PointCandidateSet;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.MapUtil;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.util.MathUtils;
import org.urbcomp.cupid.db.util.SolverUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class AMM {
    /**
     * 匹配轨迹
     */
    private MapMatchedTrajectory matchedTrajectory;
    /**
     * 匹配长度
     */
    private double matchedLength = 0;
    /**
     * 路网
     */
    private final RoadNetwork roadNetwork;
    /**
     * GPS点周围被划定为候选点的范围
     */
    private static double radius;
    /**
     * 位置正态分布的参数
     */
    private static double positionErrorSigma;
    /**
     * 优化函数中的惩罚项1
     */
    private static double punishFactor1;
    /**
     * 优化函数中的惩罚项2
     */
    private static double punishFactor2;
    /**
     * 最低得分
     */
    private double scoreThreshold;
    /**
     * 候选点平均速度和最大速度之差
     */
    private static double deltaV;
    /**
     * 最短匹配路径中的与概率有关的惩罚项
     */
    private double eta;
    /**
     * 速度误差列表，用来更新参数
     */
    private List<Double> velocityErrorList;
    /**
     * 得分列表，用来更新参数
     */
    private List<Double> scoreList;
    /**
     * 候选点个数，用来更新参数
     */
    private List<Integer> numOfPointsList;

    /**
     * 累积的匹配长度，用来更新参数
     */
    private double accMatchedLength = 0;

    public AMM(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        radius = 50.0;
        positionErrorSigma = 20.0;
        scoreThreshold = 0.1;
        punishFactor1 = 1.0;
        punishFactor2 = 1.0;
        deltaV = 5.0;
        eta = 0.0;
    }

    public double mapMatch(Trajectory traj, int id) {
        List<Double> velocity = new ArrayList<>();
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        List<GPSPoint> gpsPointList = traj.getGPSPointList();
        List<PointCandidateSet> pcSetList = generatePCSet(gpsPointList);

        PointCandidateSet prevPCSet = null;
        int consecutiveDeletions = 0;

        // 1. 计算得分和最低得分限制
        System.out.println("calculate score");
        List<Boolean> connectList = new ArrayList<>(pcSetList.size());
        scoreList = new ArrayList<>();
        numOfPointsList = new ArrayList<>();
        for (PointCandidateSet pcSet : pcSetList) {
            pcSet.v0 = calculateMinimumVelocity(velocity, prevPCSet, pcSet);
            pcSet.prevPCSet = prevPCSet;
            boolean connected = pcSet.calculateScores(roadNetwork, positionErrorSigma, deltaV);
            if (connected) {
                pcSet.score = SolverUtils.maximizeScore(pcSet, punishFactor1, punishFactor2, eta);
                // 记录得分和对应候选点的个数，方便后续更新参数
                scoreList.add(pcSet.score);
                numOfPointsList.add(pcSet.getCandidatePoints().size());
                connectList.add(true);
//                System.out.println("score:" + pcSet.score);
                prevPCSet = pcSet;
            } else {
                connectList.add(false);
            }
        }
        scoreThreshold = MathUtils.calculateMeanMinus3Std(scoreList);
        System.out.println("score threshold: " + scoreThreshold);
        // 2. 剔除低质量GPS点
        for (int i = 0; i < pcSetList.size(); i++) {
            PointCandidateSet pcSet = pcSetList.get(i);
            Boolean connected = connectList.get(i);
            if (pcSet.score > scoreThreshold && connected) {
                if (consecutiveDeletions > 2) pcSet.backward(roadNetwork, eta);
                consecutiveDeletions = 0;
            } else {
                consecutiveDeletions++;
            }
        }
        // 3. 获取匹配结果
        int size = pcSetList.size() - 1;
        List<Path> matchedPath = null;
        while (size > 0 && matchedPath == null) {
            matchedPath = generatePath(pcSetList.get(size), traj);
            size--;
        }
        if (matchedPath == null) {
            System.out.println("no match path!");
            return 0.0;
        }
        // 4. 更新模型参数
        updateParams(id);
        return matchedLength;
    }

    private List<Path> generatePath(PointCandidateSet pointCandidateSet, Trajectory trajectory) {
        System.out.println("backtrace");
        // 速度误差集合
        velocityErrorList = new ArrayList<>();
        // 从最后一个点开始回溯
        CandidatePoint currPoint = null;
        double minPathScore = Double.MAX_VALUE;
        // 匹配点集合
        List<MapMatchedPoint> matchedPointList = new ArrayList<>();
        // 匹配路径集合
        List<Path> matchedPath = new ArrayList<>();
        // 当前回溯点的 PointCandidateSet
        PointCandidateSet currPointCandidateSet = pointCandidateSet;
        // 当前回溯点的 attributesMap
        Map<CandidatePoint, CandidateAttributes> currAttributesMap = pointCandidateSet.getAttributesMap();
        // 确定第一个回溯点：距离分数最短
        for (CandidatePoint candidatePoint : pointCandidateSet.getCandidatePoints()) {
            double pathScore = currAttributesMap.get(candidatePoint).len - currAttributesMap.get(candidatePoint).accProb * eta;
//            System.out.println("accumulative prob:" + currAttributesMap.get(candidatePoint).getAccProb());
//            System.out.println("candidate path score:" + pathScore);
            if (pathScore < minPathScore) {
                minPathScore = pathScore;
                currPoint = candidatePoint;
            }
        }
//        System.out.println("select point: " + currPoint);
        if (currPoint == null) return null;
        // 记录当前PointSet的得分
//        scoreList.add(currPointCandidateSet.score);
        // 记录当前速度误差
        velocityErrorList.add(Math.abs(currPointCandidateSet.v0 - currAttributesMap.get(currPoint).velocity));
        // 获取该回溯点的 CandidateAttributes
        CandidateAttributes currPointAttributes = currAttributesMap.get(currPoint);
        // 添加到匹配点集合中
        matchedPointList.add(new MapMatchedPoint(currPointCandidateSet.getObservation(), currPoint));
        // 将该回溯点的长度记为匹配长度
        matchedLength = currPointAttributes.len;
        System.out.println("selected last point: " + currPointAttributes);
        // 更新累积匹配长度
        accMatchedLength += matchedLength;
        while (currPointAttributes.pre != null) {
            // 添加到匹配路径中
            matchedPath.add(currPointAttributes.path);
            // 向前回溯
            currPoint = currPointAttributes.pre;
            // 获取前一个 PointCandidateSet
            currPointCandidateSet = currPointCandidateSet.prevPCSet;
            // 获取前一个 AttributesMap
            currAttributesMap = currPointCandidateSet.attributesMap;
            // 获取前一个 CandidateAttribute
            currPointAttributes = currAttributesMap.get(currPoint);
            // 记录当前速度误差
            velocityErrorList.add(Math.abs(currPointCandidateSet.v0 - currPointAttributes.velocity));
            // 记录当前匹配点
            matchedPointList.add(new MapMatchedPoint(currPointCandidateSet.getObservation(), currPoint));
        }

        Collections.reverse(matchedPointList);
        Collections.reverse(matchedPath);

        matchedTrajectory = new MapMatchedTrajectory(trajectory.getTid(), trajectory.getOid(), matchedPointList);
        return matchedPath;
    }

    private void updateParams(int id) {
        System.out.println("update params");
        double newPositionErrorSigma = 0;
        List<MapMatchedPoint> mmPtList = matchedTrajectory.getMmPtList();
        int size = mmPtList.size();
        for (MapMatchedPoint mmPoint : mmPtList) {
            CandidatePoint candidatePoint = mmPoint.getCandidatePoint();
            GPSPoint gpsPoint = mmPoint.getRawPoint();
            newPositionErrorSigma += Math.pow(MapUtil.calculateDistance(candidatePoint, gpsPoint), 2.0);
        }
        newPositionErrorSigma = Math.sqrt(newPositionErrorSigma / mmPtList.size());
        // positionSigma的更新
        positionErrorSigma = (id * positionErrorSigma + newPositionErrorSigma) / (id + 1);
        // TODO:参数更新有问题
        // deltaV的更新
        deltaV = MathUtils.calculate5thPercentile(velocityErrorList);
        // punishFactor1的更新
        punishFactor1 = 10 * scoreThreshold;
        // punishFactor2的更新
        List<Double> doubleNumOfPointsList = numOfPointsList.stream().map(Integer::doubleValue).collect(Collectors.toList());
        punishFactor2 *= 1 - MathUtils.signFun(MathUtils.calculateCorrelation(scoreList, doubleNumOfPointsList)) * 0.05;
        // eta的更新
        eta = ((id + 1) * eta) / (id + 2) + accMatchedLength / ((id + 1) * size);
    }

    public double calculateMinimumVelocity(List<Double> velocity, PointCandidateSet prev, PointCandidateSet curr) {
        if (velocity.stream().allMatch(element -> element == 5)) return 5.0;
        velocity.remove(0);
        double currVelocity = MapUtil.calculateGPSPointVelocity(prev.getObservation(), curr.getObservation());
        velocity.add(currVelocity);
        double sum = velocity.stream().mapToDouble(Double::doubleValue).sum();
        return sum / velocity.size();
    }

    private List<PointCandidateSet> generatePCSet(List<GPSPoint> gpsPointList) {
        return roadNetwork.generatePCSet(gpsPointList, roadNetwork, radius);
    }

    public MapMatchedTrajectory getMatchedTraj() {
        return matchedTrajectory;
    }

    public double getMatchedLength() {
        return matchedLength;
    }
}
