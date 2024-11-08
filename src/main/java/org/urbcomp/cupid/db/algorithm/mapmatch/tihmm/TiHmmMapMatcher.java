/*
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.algorithm.mapmatch.tihmm;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.DynamicWeightAdjuster;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.FixedWeightAdjuster;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.GeoFunctions;
import scala.Tuple2;

import java.util.*;

public class TiHmmMapMatcher {
    /**
     * emission P用正态分布函数来模拟，sigma为正态分布的概率函数参数
     */
    private static final double measurementErrorSigma = 50.0;
    /**
     * transition p 的指数概率函数参数
     */
    private static final double transitionProbabilityBeta = 5.0;

    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    protected final AbstractManyToManyShortestPath pathAlgo;
    private final WindowBearing windowBearing = new WindowBearing();

    public TiHmmMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    /**
     * 实现抽象类的map match 方法
     *
     * @param traj 原始轨迹
     * @return map match后的轨迹
     */
    public MapMatchedTrajectory mapMatch(Trajectory traj) throws AlgorithmExecuteException {

        List<SequenceState> seq = this.computeViterbiSequence(traj.getGPSPointList());
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

    /**
     * 建立一个time step
     *
     * @param pt 原始轨迹点
     * @return timestep
     */
    private TimeStep createTimeStep(GPSPoint pt, int index) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                measurementErrorSigma,
                index
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }

    /**
     * 计算一个 Viterbi sequence
     *
     * @param ptList 原始轨迹ptList
     * @return 保存了每一步step的所有状态
     */
    private List<SequenceState> computeViterbiSequence(List<GPSPoint> ptList)
            throws AlgorithmExecuteException {
        List<SequenceState> seq = new ArrayList<>();
        final HmmProbabilities probabilities = new HmmProbabilities(
                measurementErrorSigma,
                transitionProbabilityBeta
        );
        TiViterbi viterbi = new TiViterbi();
        DynamicWeightAdjuster dynamicWeightAdjuster = new DynamicWeightAdjuster();
        TimeStep preTimeStep = null;
        int idx = 0;
        int nbPoints = ptList.size();// 点的数量
        while (idx < nbPoints) {
            windowBearing.addPoint(ptList.get(idx));
            TimeStep timeStep = this.createTimeStep(ptList.get(idx), idx);//轨迹点+候选点集
            if (timeStep == null) {//没有候选点
                seq.addAll(viterbi.computeMostLikelySequence()); //计算之前最有可能的序列
                seq.add(new SequenceState(null, ptList.get(idx))); //添加新状态
                viterbi = new TiViterbi();
                preTimeStep = null;
            } else {
                if (preTimeStep != null) {

                    // 找最短路径
                    Set<CandidatePoint> startPoints = new HashSet<>(preTimeStep.getCandidates());
                    Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());
                    Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                            startPoints,
                            endPoints
                    );

                    // 处理观测点向后偏移
                    processBackward(preTimeStep, timeStep, viterbi, paths, probabilities);

                    //计算观测概率
                    this.computeEmissionProbabilities(timeStep, probabilities);
                    //计算转移概率
                    this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                    //计算方向概率
//                    this.adjustWithDirection(timeStep, preTimeStep, paths, probabilities);

                    //计算维特比
                    viterbi.nextStep(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities(),
                            timeStep.getTransitionLogProbabilities(),
                            dynamicWeightAdjuster
                    );
                } else {
                    //第一个点初始化概率
                    this.computeEmissionProbabilities(timeStep, probabilities);//计算观测概率
                    viterbi.startWithInitialObservation(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities()
                    );
                }
                if (viterbi.isBroken) {
                    seq.addAll(viterbi.computeMostLikelySequence());
                    viterbi = new TiViterbi();
                    viterbi.startWithInitialObservation(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities()
                    );
//                    System.out.println("Ti-HMM: Viterbi algorithm is broken at index " + idx + ptList.get(idx));
//                    System.out.println();
                }
                preTimeStep = timeStep;
            }
            idx += 1;
        }
        if (seq.size() < nbPoints) { //添加最后的
            seq.addAll(viterbi.computeMostLikelySequence());
        }
        return seq;
    }

    // 处理观测点向后偏移的情况
    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep, TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths, HmmProbabilities probabilities) {
        CandidatePoint preCandiPt = StreamMapMatcher.findMaxValuePoint(viterbi.message);
        int roadSegmentId = preCandiPt.getRoadSegmentId();
        List<CandidatePoint> curCandidates = curTimeStep.getCandidates();
        boolean isMatch = false;
        for (CandidatePoint curCandiPt : curCandidates) {
            if (curCandiPt.getRoadSegmentId() == roadSegmentId && curCandiPt.getOffsetInMeter() < preCandiPt.getOffsetInMeter()) {
                RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(roadSegmentId);
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                double speed = path.getLengthInMeter() * 1000 / (curTimeStep.getObservation().getTime().getTime() - preTimeStep.getObservation().getTime().getTime());
                if (speed > 34) {
                    double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
                    if (disBtwCurAndPer < 20) {
                        isMatch = true;
                        break;
                    }
                }
            }
        }
        if (isMatch) {
            curTimeStep.addCandidate(preCandiPt);
        }
    }


    public void adjustWithDirection(TimeStep currTimeStep, TimeStep preTimeStep, Map<RoadNode, Map<RoadNode, Path>> paths, HmmProbabilities probabilities) {
        if (!windowBearing.getChange()) {
//            System.out.println(windowBearing.getChange() + " " + windowBearing.getChangeScore() + " " + currTimeStep.getObservation());
        } else {
            GPSPoint currObPoint = currTimeStep.getObservation();
            GPSPoint preObPoint = preTimeStep.getObservation();
            double obBearing = GeoFunctions.getBearing(preObPoint.getLng(), preObPoint.getLat(), currObPoint.getLng(), currObPoint.getLat());
            double speed = GeoFunctions.getDistanceInM(preObPoint.getLng(), preObPoint.getLat(), currObPoint.getLng(), currObPoint.getLat()) * 1000 / (currObPoint.getTime().getTime() - preObPoint.getTime().getTime());
            if (speed < 2.0) {
//                System.out.println(windowBearing.getChange() + " " + windowBearing.getChangeScore() + " " + "speed: "+ speed + " " + currTimeStep.getObservation());
            } else {
                for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : currTimeStep.getTransitionLogProbabilities().entrySet()) {
                    Tuple2<CandidatePoint, CandidatePoint> key = entry.getKey();
                    RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                            key._1.getRoadSegmentId()
                    );
                    RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                            key._2.getRoadSegmentId()
                    );
                    Path subPath = paths.get(startRoadSegment.getEndNode())
                            .get(endRoadSegment.getStartNode());
                    Path path = pathAlgo.getCompletePath(key._1, key._2, subPath);
                    double candidateBearing = path.calDisWeightDirection(roadNetwork);
                    double angleDifference = Math.abs(obBearing - candidateBearing);
                    if (angleDifference > 180) {
                        angleDifference = 360 - angleDifference;
                    }
                    currTimeStep.getTransitionLogProbabilities().put(key, currTimeStep.getTransitionLogProbabilities().get(key) + probabilities.directionLogProbability(angleDifference));
                }
//                System.out.println("true direction: " + windowBearing.getChangeScore() + " " + currTimeStep.getObservation());
            }
        }
    }


    /**
     * 根据time step和概率分布函数计算emission P
     *
     * @param timeStep    timeStep
     * @param probability 建立好的概率分布函数
     */
    private void computeEmissionProbabilities(TimeStep timeStep, HmmProbabilities probability) {
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            final double dist = candiPt.getErrorDistanceInMeter();
            timeStep.addEmissionLogProbability(candiPt, probability.emissionLogProbability(dist));
        }
    }

    /**
     * 计算之前timeStep到当前timeStep的转移概率
     *
     * @param prevTimeStep  之前的timestep
     * @param timeStep      当前的timestep
     * @param probabilities 建立好的概率分布函数
     * @param paths         候选点间最短路径
     */
    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths
    ) throws AlgorithmExecuteException {
        //观测点的距离
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );

        // 计算候选点转移概率
        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    preCandiPt.getRoadSegmentId()
            );
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {

                // 两个候选点相同，将概率置为极大
                if (preCandiPt == curCandiPt) {
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            probabilities.transitionLogProbability(0.0, 0.0)
                    );
                    continue;
                }

                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                if (path.getLengthInMeter() != Double.MAX_VALUE) {
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist)
                    );
                }
            }
        }
    }

}
