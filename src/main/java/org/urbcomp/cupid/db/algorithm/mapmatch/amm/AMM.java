package org.urbcomp.cupid.db.algorithm.mapmatch.amm;

import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.*;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.MapUtil;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.util.SolverUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AMM {

    private List<Candidate> matched_list;
    /**
     * 匹配长度
     */
    private double matchedLength = 0;
    /**
     * 路网
     */
    private static RoadNetwork roadNetwork;
    /**
     * GPS点周围被划定为候选点的范围
     */
    private static double radius;
    /**
     * 位置正态分布的参数
     */
    private static double positionSigma;
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
    private final double scoreThreshold;
    /**
     * 候选点平均速度和最大速度之差
     */
    private static double deltaV;

    public AMM(RoadNetwork roadNetwork) {
        AMM.roadNetwork = roadNetwork;
        radius = 50.0;
        positionSigma = 20.0;
        scoreThreshold = 0.1;
        punishFactor1 = 1.0;
        punishFactor2 = 2.0;
        deltaV = 5.0;
    }

    public double mapMatch(Trajectory trajectory, int id) {
        List<Double> velocity = new ArrayList<>();
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        velocity.add(5.0);
        List<GPSPoint> trajectoryList = trajectory.getGPSPointList();
        List<AMMGPSPoint> gpsPointList = convertGPSPointToAMM(trajectoryList);
        List<PointsSet> trackList = generateSequence(gpsPointList);

        PointsSet previous = null;
        int consecutiveDeletions = 0;

        System.out.println("calculate score");
        for (PointsSet current : trackList) {
            current.v0 = calculateMinimumVelocity(velocity, previous, current);
            if (current.setScore(previous)) {
                current.score = SolverUtils.maximizeScore(current.getCandidates(), punishFactor1, punishFactor2);
                if (current.score > scoreThreshold) {
                    previous = current;
                    if (consecutiveDeletions > 2) current.backward();
                    consecutiveDeletions = 0;
                } else consecutiveDeletions++;
            } else consecutiveDeletions++;
        }

        System.out.println("generate path");
        int location = trackList.size() - 1;
        List<Path> matchedPath = null;
        while (location > 0 && matchedPath == null) {
            matchedPath = generatePath(trackList.get(location));
            location--;
        }
        if (matchedPath == null) {
            System.out.println("no match path!");
            return 0.0;
        }
        // 4. 更新模型参数
        updateParams(id);
        return matchedLength;
    }

    private List<Path> generatePath(PointsSet final_point) {
        Candidate last_point = null;
        double min_length = Double.MAX_VALUE;
        // 距离最短的点作为回溯的第一个点
        for (Candidate candidate : final_point.getCandidates()) {
            if (candidate.getMinLength() < min_length) last_point = candidate;
        }
        if (last_point == null) {
            return null;
        }
        matched_list = new ArrayList<>();
        matched_list.add(last_point);
        matchedLength = last_point.min_length;
        List<Path> result = new ArrayList<>();
        // 向前回溯，记录最优路径和最优的上一个候选点
        while (last_point.best_previous != null) {
            result.add(last_point.best_path);
            last_point = last_point.best_previous;
            matched_list.add(last_point);
        }
        // 逆序输出
        Collections.reverse(matched_list);
        Collections.reverse(result);
        return result;
    }

    private void updateParams(int id) {
        System.out.println("update params");
        double newPosSigma = 0;
        for (Candidate candidate : matched_list) {
            newPosSigma += Math.pow(MapUtil.calculateDistance(candidate.candidate, candidate.parent.getObservation()), 2.0);
        }
        newPosSigma = Math.sqrt(newPosSigma / matched_list.size());
        positionSigma = (id * positionSigma + newPosSigma) / (id + 1);
    }

    public double calculateMinimumVelocity(List<Double> velocity, PointsSet prev, PointsSet curr) {
        if (velocity.stream().allMatch(element -> element == 5)) return 5.0;
        velocity.remove(0);
        double currVelocity = MapUtil.calculateGPSPointVelocity(prev.getObservation(), curr.getObservation());
        velocity.add(currVelocity);
        double sum = velocity.stream().mapToDouble(Double::doubleValue).sum();
        return sum / velocity.size();
    }

    private List<PointsSet> generateSequence(List<AMMGPSPoint> GPSPointList) {
        return roadNetwork.generateSequence(GPSPointList, roadNetwork, radius);
    }

    private List<AMMGPSPoint> convertGPSPointToAMM(List<GPSPoint> gpsPointList) {
        ArrayList<AMMGPSPoint> trackList = new ArrayList<>();
        for (GPSPoint gpsPoint : gpsPointList) {
            trackList.add(new AMMGPSPoint(gpsPoint.getTime(), gpsPoint.getLng(), gpsPoint.getLat()));
        }
        return trackList;
    }

    public static double getDeltaV() {
        return deltaV;
    }

    public static RoadNetwork getRoadNetwork() {
        return roadNetwork;
    }

    public List<Candidate> getMatchedList() {
        return matched_list;
    }
}
