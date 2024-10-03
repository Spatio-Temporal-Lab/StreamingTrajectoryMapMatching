package org.urbcomp.cupid.db.algorithm.mapmatch.aomm;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AStarShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.GeoFunctions;

import scala.Tuple3;


import java.util.*;

// 复现论文“A Self-adjusting Online Map Matching Method”
public class AommMapMatcher {
    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    protected final AStarShortestPath aStarShortestPath;

    private final WindowBearing windowBearing = new WindowBearing();

    public AommMapMatcher(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.aStarShortestPath = new AStarShortestPath(roadNetwork);
    }

    private SequenceState originState;

    private TiViterbi originViterbi;

    private int recallNum;

    public MapMatchedTrajectory AommMapMatch(Trajectory traj) throws AlgorithmExecuteException {

        // 使用 LinkedList 来保存过去四个 TimeStep
        LinkedList<TimeStep> preTimeSteps = new LinkedList<>();
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            recallNum = 0;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeSteps, viterbi);
            seq = tuple2._1();
            TimeStep currentTimeStep = tuple2._2();
            viterbi = tuple2._3();

            // 更新 preTimeSteps 列表
            if (!seq.get(seq.size() - 1).getState().isSkip()) {
                preTimeSteps.addLast(currentTimeStep);
            }
            if (preTimeSteps.size() > 4) {
                preTimeSteps.removeFirst();
            }
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


    /**
     * 计算一个 Viterbi sequence
     *
     * @param point 原始轨迹ptList
     * @return 保存了每一步step的所有状态
     */
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point, List<SequenceState> seq, LinkedList<TimeStep> preTimeStepList, TiViterbi viterbi)
            throws AlgorithmExecuteException {
        windowBearing.addPoint(point);
        TimeStep timeStep = this.createTimeStep(point);//轨迹点+候选点集
        TimeStep preTimeStep = null;
        if (!preTimeStepList.isEmpty()) {
            preTimeStep = preTimeStepList.getLast();
        }

        // strategy 1
        if (timeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, point)); //添加新状态
        }
        // strategy 1

        else {
            if (preTimeStep != null) {
                final double linearDist = GeoFunctions.getDistanceInM(
                        preTimeStep.getObservation(),
                        timeStep.getObservation()
                );

                // strategy 2
                if (linearDist < 3 && recallNum == 0) {
                    CandidatePoint skipPoint = new CandidatePoint();
                    skipPoint.setSkip(true);
                    seq.add(new SequenceState(skipPoint, point)); //添加新状态
                    return Tuple3.apply(seq, preTimeStep, viterbi);
                }
                // strategy 2

                else {
                    // 计算观测概率
                    this.computeEmissionProbabilities(timeStep);

                    // 计算转移概率
                    this.computeTransitionProbabilities(preTimeStep, timeStep);

                    // 计算维特比
                    viterbi.nextStep(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities(),
                            timeStep.getTransitionLogProbabilities()
                    );

                    // strategy 3
                    CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
                    timeStep.setMatch(maxPoint);
                    if (preTimeStep.getMatch() != null) {
                        Path path = aStarShortestPath.findShortestPath(preTimeStep.getMatch(), maxPoint);
                        double pathDistance = path.getLengthInMeter();
                        if (pathDistance > 100) {
                            if (recallNum == 0) {
                                originState = new SequenceState(maxPoint, point);
                                originViterbi = viterbi;
                            }
                            preTimeStepList.removeLast();
                            if (!preTimeStepList.isEmpty() && !viterbi.isBroken) {
                                recallNum++;
                                return computeViterbiSequence(point, seq, preTimeStepList, viterbi);
                            }
                            else if (!originViterbi.isBroken){
                                seq.add(originState);
                                return Tuple3.apply(seq, timeStep, originViterbi);
                            }
                        }
                    }
                    // strategy 3

                }
            } else {
                //第一个点初始化概率
                this.computeEmissionProbabilities(timeStep);//计算观测概率
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            }
            if (viterbi.isBroken) {
                seq.add(viterbi.computeMostLikelySequence().get(viterbi.computeMostLikelySequence().size() - 1));
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            } else {
                CandidatePoint maxPoint = AommMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
                seq.add(new SequenceState(maxPoint, point));
                timeStep.setMatch(maxPoint);
            }
            preTimeStep = timeStep;
        }

        // strategy 3
        int j = 2;
        for (int i = 0; i < recallNum;) {
            if (seq.get(seq.size() - j).getState().isSkip()) {
                j++;
            }
            else {
                seq.remove(seq.size() - j);
                i++;
            }
        }
        for (int i = 0; i < recallNum; i++) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(seq.size() - j, new SequenceState(skipPoint, point)); //添加新状态
        }
        // strategy 3

        return Tuple3.apply(seq, preTimeStep, viterbi);
    }

    // 寻找匹配点
    public static CandidatePoint findMaxValuePoint(Map<CandidatePoint, Double> map) {
        CandidatePoint maxPoint = null;
        double maxProb = Double.MIN_VALUE;
        if (map == null) {
            return maxPoint;
        }
        for (CandidatePoint candiPt : map.keySet()) {
            if (maxPoint == null) {
                maxPoint = candiPt;
                maxProb = map.get(candiPt);
            } else if (map.get(candiPt) > maxProb) {
                maxPoint = candiPt;
                maxProb = map.get(candiPt);
            }
        }
        return maxPoint;
    }

    // 创建时间步，初始化候选点
    private TimeStep createTimeStep(GPSPoint pt) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                50.0
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }

    /**
     * 计算观测概率
     *
     * @param timeStep    timeStep
     */
    private void computeEmissionProbabilities(TimeStep timeStep) {
        double distSum = 0;
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            distSum += 1 / candiPt.getErrorDistanceInMeter();
        }
        double distMean = distSum / timeStep.getCandidates().size();
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            final double dist = 1 / candiPt.getErrorDistanceInMeter();
            timeStep.addEmissionLogProbability(candiPt, Math.log(dist / distMean));
        }
    }

    /**
     * 计算转移概率
     *
     * @param prevTimeStep  之前的timestep
     * @param timeStep      当前的timestep
    */
    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep
    ) throws AlgorithmExecuteException {
        //观测点的距离
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );

        // 计算候选点转移概率
        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            double sum = 0;
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                Path path = aStarShortestPath.findShortestPath(preCandiPt, curCandiPt);
                double pathDistance = path.getLengthInMeter();
                sum += 1 / Math.abs(linearDist - pathDistance);
            }

            double mean = sum / timeStep.getCandidates().size();
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                Path path = aStarShortestPath.findShortestPath(preCandiPt, curCandiPt);
                double pathDistance = path.getLengthInMeter();
                double transitionprob = 1 / Math.abs(linearDist - pathDistance) / mean;

                if (path.getLengthInMeter() != Double.MAX_VALUE) {
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            Math.log(transitionprob)
                    );
                }
            }
        }
    }
}
