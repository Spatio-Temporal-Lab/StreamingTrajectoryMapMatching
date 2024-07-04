package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.urbcomp.cupid.db.algorithm.kalman.KalmanFilter;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
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
import scala.Tuple3;

import java.util.*;

public class StreamMapMatcher {
    /**
     * emission P用正态分布函数来模拟，sigma为正态分布的概率函数参数
     */
    private static final double measurementErrorSigma = 50.0;
    /**
     * transition p 的指数概率函数参数
     */
    private static final double transitionProbabilityBeta = 2.0;

    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    protected final AbstractManyToManyShortestPath pathAlgo;
    private final KalmanFilter kalmanFilter;

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
        double[][] F = new double[][]{
                {1, 0, 1, 0},
                {0, 1, 0, 1},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };
        double[][] H = new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };
        double pre_var = 15 * 15;
        double gps_var = 100 * 100;
        this.kalmanFilter = new KalmanFilter(F, H, gps_var, pre_var);
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj, double alpha, double beta) throws AlgorithmExecuteException {

        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);

            double[] estimate = kalmanFilter.process(p.getX(), p.getY(), p.getTime());

            GPSPoint filterPoint = new GPSPoint(p.getTime(), estimate[0], estimate[1]);

            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeStep, viterbi, filterPoint, i, alpha, beta);
            seq = tuple2._1();
            preTimeStep = tuple2._2();
            viterbi = tuple2._3();
        }
//        if (seq.size() < size) { //添加最后的
//            seq.addAll(viterbi.computeMostLikelySequence());
//        }
//        System.out.println("seq size: " + seq.size());
//        System.out.println("traj size: " + traj.getGPSPointList().size());
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
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point, List<SequenceState> seq, TimeStep preTimeStep, TiViterbi viterbi, GPSPoint filterPoint, int index, double alpha, double beta)
            throws AlgorithmExecuteException {
        TimeStep timeStep = this.createTimeStep(point);//轨迹点+候选点集
        TimeStep filterStep = this.createTimeStep(filterPoint);
        if (timeStep == null) {
//            seq.addAll(viterbi.computeMostLikelySequence()); //计算之前最有可能的序列
            seq.add(new SequenceState(null, point)); //添加新状态
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
                processBackward(preTimeStep, timeStep, viterbi, paths);

                //计算观测概率
                this.computeEmissionProbabilities(timeStep, probabilities);

                //计算转移概率
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                //计算维特比
                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities()
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
//                seq.addAll(viterbi.computeMostLikelySequence());
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            }
            CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
            kalmanFilter.update(maxPoint.getX(),maxPoint.getY(),point.getTime());
            seq.add(new SequenceState(maxPoint, point));
            preTimeStep = timeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
    }

    // 处理观测点向后偏移的情况
    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep, TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths) {
        int roadSegmentId = StreamMapMatcher.findMaxValuePoint(viterbi.message).getRoadSegmentId();
        List<CandidatePoint> curCandidates = curTimeStep.getCandidates();
        int i = 0;
        for (; i < curCandidates.size(); i++) {
            if (curCandidates.get(i).getRoadSegmentId() == roadSegmentId) {
                break;
            }
        }
        if (i != curCandidates.size()) {
            for (CandidatePoint preCandiPt : preTimeStep.getCandidates()) {
                if (preCandiPt.getRoadSegmentId() == roadSegmentId) {

                    RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                            preCandiPt.getRoadSegmentId()
                    );
                    CandidatePoint curCandiPt = curCandidates.get(i);
                    if (preCandiPt != curCandiPt) {
                        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                                curCandiPt.getRoadSegmentId()
                        );
                        Path subPath = paths.get(startRoadSegment.getEndNode())
                                .get(endRoadSegment.getStartNode());
                        Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                        double speed = path.getLengthInMeter() * 1000 / (curTimeStep.getObservation().getTime().getTime() - preTimeStep.getObservation().getTime().getTime());
                        if (speed > 34) {
                            double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
                            if (disBtwCurAndPer < 20) {
                                curTimeStep.addCandidate(preCandiPt);
                            }
                        }
                    }
                }
            }
        }
    }

    public void adjustEmissionProbabilities(TimeStep timeStep, TimeStep filterTimeStep, double alpha, double beta) {
        for (Map.Entry<CandidatePoint, Double> candidatePointDoubleEntry : timeStep.getEmissionLogProbabilities().entrySet()) {
            long roadId = candidatePointDoubleEntry.getKey().getRoadSegmentId();
            for (Map.Entry<CandidatePoint, Double> filterCandidatePointDoubleEntry : filterTimeStep.getEmissionLogProbabilities().entrySet()) {
                long filterRoadId = filterCandidatePointDoubleEntry.getKey().getRoadSegmentId();
                if (roadId == filterRoadId) {
                    candidatePointDoubleEntry.setValue(candidatePointDoubleEntry.getValue() * alpha + filterCandidatePointDoubleEntry.getValue() * beta);
                }
            }
        }

    }

    // 寻找匹配点
    public static CandidatePoint findMaxValuePoint(Map<CandidatePoint, Double> map) {
        CandidatePoint maxPoint = null;
        double maxProb = Double.MIN_VALUE;
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
     * @param paths 候选点间最短路径
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

    // 创建时间步，初始化候选点
    private TimeStep createTimeStep(GPSPoint pt) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                measurementErrorSigma
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }


}
