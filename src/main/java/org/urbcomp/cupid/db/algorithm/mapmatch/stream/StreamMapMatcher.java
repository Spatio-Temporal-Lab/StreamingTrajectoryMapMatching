package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.urbcomp.cupid.db.algorithm.kalman.AdaptiveKalmanFilter;
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
    private static final double transitionProbabilityBeta = 2;

    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    public List<GPSPoint> filteredPoints;
    private int location = 0;

    protected final AbstractManyToManyShortestPath pathAlgo;
    private final AdaptiveKalmanFilter kalmanFilter;

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
        this.kalmanFilter = new AdaptiveKalmanFilter(F, H, gps_var, pre_var);
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj, double alpha, double beta) throws AlgorithmExecuteException {
        filteredPoints = new ArrayList<>();
        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);

            double[] estimate = kalmanFilter.process(p.getX(), p.getY(), p.getTime());

            GPSPoint filterPoint = new GPSPoint(p.getTime(), estimate[0], estimate[1]);
            filteredPoints.add(filterPoint);

            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeStep, viterbi, filterPoint, alpha, beta);
            seq = tuple2._1();
            preTimeStep = tuple2._2();
            viterbi = tuple2._3();
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
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point, List<SequenceState> seq, TimeStep preTimeStep, TiViterbi viterbi, GPSPoint filterPoint, double alpha, double beta)
            throws AlgorithmExecuteException {
        TimeStep timeStep = this.createTimeStep(point);//轨迹点+候选点集
        TimeStep filterStep = this.createTimeStep(filterPoint);
        if (timeStep == null) {
            seq.add(new SequenceState(null, point)); //添加新状态
            viterbi = new TiViterbi();
            preTimeStep = null;

        } else {
            this.computeEmissionProbabilities(timeStep, probabilities);//计算观测概率
//            if (filterStep != null && index > 3) {
//                this.computeEmissionProbabilities(filterStep, probabilities);
//                adjustEmissionProbabilities(timeStep, filterStep, alpha, beta);
//            }
            if (preTimeStep == null) { //第一个点初始化概率
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        beta
                );
            } else {//计算转移概率
                int maxProbPointRSId = StreamMapMatcher.findMaxValuePoint(viterbi.message).getRoadSegmentId();//找到最大概率的候选点
//                if (alpha == 1 || index <= 3 || filterStep != null){
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, maxProbPointRSId);
//                }else {
//                    this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, maxProbPointRSId, filterStep);
//                }

                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities(),
                        beta
                );//计算维特比
            }
            if (viterbi.isBroken) {
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        beta
                );
            }
            CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
            if (!locChange(maxPoint)) {
//                kalmanFilter.updateByReal(maxPoint.getX(), maxPoint.getY());
                kalmanFilter.updateNoiseCovariances(maxPoint.getX(), maxPoint.getY());
            }
            seq.add(new SequenceState(maxPoint, point));
            preTimeStep = timeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
    }

    private boolean locChange(CandidatePoint maxPoint) {
        if (location == 0) {
            location = maxPoint.getRoadSegmentId();
            return false;
        } else {
            if (maxPoint.getRoadSegmentId() == location) {
                return false;
            } else {
                location = maxPoint.getRoadSegmentId();
                return true;
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
     * 计算之前timeStep到当前timeStep的概率
     *
     * @param prevTimeStep  之前的timestep
     * @param timeStep      当前的timestep
     * @param probabilities 建立好的概率分布函数
     */
    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            int maxProbPointRSId
    ) throws AlgorithmExecuteException {
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );//观测点的距离

        Set<CandidatePoint> startPoints = new HashSet<>(prevTimeStep.getCandidates());
        Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());
        Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                startPoints,
                endPoints
        );//找最短路径

        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    preCandiPt.getRoadSegmentId()
            );
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
//                double speed = path.getLengthInMeter() * 1000 / (timeStep.getObservation().getTime().getTime() - prevTimeStep.getObservation().getTime().getTime());
//                if (speed > 55.6 && curCandiPt.getRoadSegmentId() == preCandiPt.getRoadSegmentId() && curCandiPt.getRoadSegmentId() == maxProbPointRSId) {
//                    double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
//                    if (disBtwCurAndPer < 20) {
//                        timeStep.addTransitionLogProbability(
//                                preCandiPt,
//                                curCandiPt,
//                                probabilities.transitionLogProbability(0.0, 0.0)
//                        );
//                    }
//                    continue;
//                }
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

    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            int maxProbPointRSId,
            TimeStep filterStep
    ) throws AlgorithmExecuteException {
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );//观测点的距离
        final double linearDist2 = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                filterStep.getObservation()
        );//观测点的距离

        Set<CandidatePoint> startPoints = new HashSet<>(prevTimeStep.getCandidates());
        Set<CandidatePoint> endFilterPoints = new HashSet<>(filterStep.getCandidates());
        Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());
        Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                startPoints,
                endPoints
        );//找最短路径
        Map<RoadNode, Map<RoadNode, Path>> paths2 = pathAlgo.findShortestPath(
                startPoints,
                endFilterPoints
        );

        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    preCandiPt.getRoadSegmentId()
            );
            Map<Integer, Double> temp = new HashMap<>();

            for (CandidatePoint curCandiPt : filterStep.getCandidates()) {
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths2.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                temp.put(curCandiPt.getRoadSegmentId(), path.getLengthInMeter());
            }

            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                double speed = path.getLengthInMeter() * 1000 / (timeStep.getObservation().getTime().getTime() - prevTimeStep.getObservation().getTime().getTime());
                if (speed > 55.6 && curCandiPt.getRoadSegmentId() == preCandiPt.getRoadSegmentId() && curCandiPt.getRoadSegmentId() == maxProbPointRSId) {
                    double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
                    if (disBtwCurAndPer < 20) {
                        timeStep.addTransitionLogProbability(
                                preCandiPt,
                                curCandiPt,
                                probabilities.transitionLogProbability(0.0, 0.0)
                        );
                    }
                    continue;
                }
                if (path.getLengthInMeter() != Double.MAX_VALUE) {
                    double tempLength = 0;
                    if (temp.containsKey(curCandiPt.getRoadSegmentId())) {
                        tempLength = temp.get(curCandiPt.getRoadSegmentId());
                    }
                    if (tempLength != 0) {
                        timeStep.addTransitionLogProbability(
                                preCandiPt,
                                curCandiPt,
                                probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist)
                        );
                    } else {
                        System.out.println("change!!");
                        timeStep.addTransitionLogProbability(
                                preCandiPt,
                                curCandiPt,
                                probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist) * 0.5 + probabilities.transitionLogProbability(tempLength, linearDist2) * 0.5
                        );
                    }
                }
            }
        }
    }


    private TimeStep createTimeStep(GPSPoint pt) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                50
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }


}
