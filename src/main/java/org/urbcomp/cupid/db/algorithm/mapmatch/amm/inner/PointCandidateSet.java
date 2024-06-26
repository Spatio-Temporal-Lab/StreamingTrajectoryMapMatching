package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;

import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.util.MapUtil;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PointCandidateSet {
    /**
     * GPS点
     */
    private final GPSPoint observation;
    /**
     * GPS点对应的候选点列表
     */
    private final List<CandidatePoint> candidatePoints;
    /**
     * 最低速度，为前面若干阶段速度的平均值
     */
    public double v0;
    /**
     * 所有候选点到起点的最短路径长度
     */
    public double minPathLength = Double.MAX_VALUE;
    /**
     * 所有候选点到起点的最小路径得分
     */
    public double minPathScore = Double.MAX_VALUE;
    /**
     * 当前PointSet的评分
     */
    public double score = Double.MIN_VALUE;
    /**
     * 上一个连接的PointSet
     */
    public PointCandidateSet prevPCSet = null;
    /**
     * 存储了每个候选点在速度、位置、方向的得分字典
     */
    public Map<CandidatePoint, CandidateScores> scoresMap = new HashMap<>();
    /**
     * 存储了每个候选点在匹配过程中使用到的属性字典
     */
    public Map<CandidatePoint, CandidateAttributes> attributesMap = new HashMap<>();

    public PointCandidateSet(GPSPoint observation, List<CandidatePoint> candidatePoints) {
        this.observation = observation;
        this.candidatePoints = candidatePoints;
    }

    public GPSPoint getObservation() {
        return observation;
    }

    public List<CandidatePoint> getCandidatePoints() {
        return candidatePoints;
    }

    public Map<CandidatePoint, CandidateAttributes> getAttributesMap() {
        return attributesMap;
    }

    public void setV0(double v0) {
        this.v0 = v0;
    }

    public void setPrevPCSet(PointCandidateSet pcSet) {
        this.prevPCSet = pcSet;
    }

    public boolean calculateScores(RoadNetwork roadNetwork, double posSigma, double deltaV) {
        double deltaPosition, positionScore, velocityScore;
        boolean connected = false;
        for (CandidatePoint current : candidatePoints) {
            // 计算位置分数
            deltaPosition = calculateDeltaPosition(current, observation);
            positionScore = CandidateScores.calculatePositionScore(deltaPosition, posSigma);
            // 计算速度分数
            velocityScore = calculateVelocityScore(roadNetwork, current, deltaV);
            // 保存分数信息
            scoresMap.put(current, new CandidateScores(positionScore, velocityScore));

            if (!isConnected(velocityScore)) continue;
            if (attributesMap.get(current).len < minPathLength) minPathLength = attributesMap.get(current).len;
            connected = true;
        }
        return connected;
    }

    private double calculateDeltaPosition(CandidatePoint current, GPSPoint observation) {
        return MapUtil.calculateDistance(current, observation);
    }

    private double calculateVelocityScore(RoadNetwork roadNetwork, CandidatePoint current, double deltaV) {
        double velocityScore;
        CandidateAttributes candidateAttributes = new CandidateAttributes();
        if (prevPCSet == null) {
            velocityScore = CandidateScores.calculateVelocityScore(v0, v0, deltaV);
            candidateAttributes.len = 0;
            candidateAttributes.accProb = 0;
        } else {
            List<CandidatePoint> prevCandidatePoints = prevPCSet.getCandidatePoints();
            // 上一个PCSet的属性
            Map<CandidatePoint, CandidateAttributes> prevPCSetAttributesMap = prevPCSet.attributesMap;

            ManyToManyShortestPath algo = new ManyToManyShortestPath(roadNetwork);
            velocityScore = 0;

            for (CandidatePoint prev : prevCandidatePoints) {
                Path shortestPath = algo.findShortestPathBetweenCandidates(prev, current);
                double score = 0;
                // 初始化距离
                double pathLen, prevLen, currLen = Double.MAX_VALUE;
                double prevAccProb;
                // 两个点之间存在路径
                if (!shortestPath.getPoints().isEmpty()) {
                    // 两个点之间的距离
                    pathLen = shortestPath.getLengthInMeter();
                    // 从第一个点到上一个点之间的距离
                    prevLen = prevPCSetAttributesMap.get(prev).len;
                    // 上一个点的累积概率
                    prevAccProb = prevPCSetAttributesMap.get(prev).accProb;
//                    System.out.println("prevAccProb: " + prevAccProb);
                    if (pathLen + prevLen < currLen) {
                        // 从第一个点到当前点之间的距离
                        currLen = pathLen + prevLen;
                        // 先记录之前的累积概率，方便后续加上当前点的概率
                        candidateAttributes.accProb = prevAccProb;
                        candidateAttributes.len = currLen;
                        candidateAttributes.pre = prev;
                        candidateAttributes.path = shortestPath;
                    }
                    // 计算当前速度
                    double velocity = pathLen /
                            (double) (observation.getTime().getTime() - prevPCSet.getObservation().getTime().getTime());
                    candidateAttributes.velocity = velocity;
                    // 计算速度分数
                    score = CandidateScores.calculateVelocityScore(velocity, v0, deltaV);
                }
                // 计算最终的速度得分
                velocityScore += score * prevPCSetAttributesMap.get(prev).prob;
            }
        }
        attributesMap.put(current, candidateAttributes);
        return velocityScore;
    }

    public boolean isConnected(double velocityScore) {
        return velocityScore > 0;
    }

    public void backward(RoadNetwork roadNetwork, double eta) {
        System.out.println("backward");
        if (prevPCSet == null) return;
        PointCandidateSet prev = prevPCSet;
        PointCandidateSet beforePrev = prev.prevPCSet;
        while (beforePrev != null) {
            double newPathScore = minPathScore;
            for (CandidatePoint current : candidatePoints) {
                CandidateAttributes currentAttributes = attributesMap.get(current);
                // 如果不使用上一个 pcSet，累积概率为减去上一个候选点概率的结果
                double accProb = attributesMap.get(current).accProb - prev.attributesMap.get(currentAttributes.pre).prob;
                for (CandidatePoint past : beforePrev.candidatePoints) {
                    ManyToManyShortestPath algo = new ManyToManyShortestPath(roadNetwork);
                    Path shortestPath = algo.findShortestPathBetweenCandidates(current, past);
                    if (!shortestPath.getPoints().isEmpty()) {
                        double pastLength = shortestPath.getLengthInMeter() + beforePrev.getAttributesMap().get(past).len;
                        double pathScore = pastLength - eta * accProb;

                        if (pathScore < newPathScore) {
                            newPathScore = pastLength;
                            currentAttributes.pre = past;
                            currentAttributes.accProb = accProb;
                            currentAttributes.len = pastLength;
                            currentAttributes.path = shortestPath;
                            currentAttributes.velocity = pastLength /
                                    (double) (observation.getTime().getTime() - beforePrev.getObservation().getTime().getTime());
                            prevPCSet = beforePrev;

                        }
                    }
                }
            }
            if (newPathScore < minPathScore) {
                minPathScore = newPathScore;
                beforePrev = beforePrev.prevPCSet;
                System.out.println("previous PCSet has changed");
            } else {
                System.out.println("No change! Stop backward!");
                break;
            }
        }
    }
}
