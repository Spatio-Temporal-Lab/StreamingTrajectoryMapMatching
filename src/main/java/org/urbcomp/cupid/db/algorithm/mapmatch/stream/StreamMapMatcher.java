package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.history.generateHistoryProb;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
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
import org.xml.sax.SAXException;
import scala.Tuple2;
import scala.Tuple3;

import javax.xml.bind.JAXBException;
import java.io.IOException;
import java.util.*;

public class StreamMapMatcher {
    /**
     * emission P用正态分布函数来模拟，sigma为正态分布的概率函数参数
     */
    private static final double measurementErrorSigma = 50.0;
    /**
     * transition p 的指数概率函数参数
     */
    private static final double transitionProbabilityBeta = 5.0;

    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    protected final AbstractManyToManyShortestPath pathAlgo;

    protected BidirectionalManyToManyShortestPath impAlgo;

    private final WindowBearing windowBearing = new WindowBearing();

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo, BidirectionalManyToManyShortestPath impAlgo) {
        this.roadNetwork = roadNetwork;
        this.impAlgo = impAlgo;
        this.pathAlgo = pathAlgo;
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj) throws AlgorithmExecuteException, JAXBException, IOException, SAXException {

        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeStep, viterbi, i);
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
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point, List<SequenceState> seq, TimeStep preTimeStep, TiViterbi viterbi, int index)
            throws AlgorithmExecuteException, JAXBException, IOException, SAXException {
        windowBearing.addPoint(point);
        TimeStep timeStep = this.createTimeStep(point);//轨迹点+候选点集
        if (timeStep == null) {
            seq.add(new SequenceState(null, point)); //添加新状态
            viterbi = new TiViterbi();
            preTimeStep = null;
        } else {
            if (preTimeStep != null) {
                // 找最短路径
                Set<CandidatePoint> startPoints = new HashSet<>(preTimeStep.getCandidates());
                Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());

                Map<RoadNode, Map<RoadNode, Path>> paths;
                // 单向 dijkstra
                if (impAlgo == null) {
                    paths = pathAlgo.findShortestPath(
                            startPoints,
                            endPoints
                    );
                } else {
                    // 双向 dijkstra
                    paths = impAlgo.findShortestPath(
                            startPoints,
                            endPoints
                    );
                }

                // 处理观测点向后偏移
//                this.processBackward(preTimeStep, timeStep, viterbi, paths);

                // 计算观测概率
                this.computeEmissionProbabilities(timeStep, probabilities);

                // 计算转移概率
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                // 根据方向修正
//                this.adjustWithDirection(timeStep, preTimeStep, paths, probabilities);

                // 计算维特比
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
                seq.add(viterbi.computeMostLikelySequence().get(viterbi.computeMostLikelySequence().size() - 1));
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            } else {
                CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
                seq.add(new SequenceState(maxPoint, point));
            }
            preTimeStep = timeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
    }

    // 处理观测点向后偏移的情况
    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep, TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths) {
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

                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                double transitionLogProbability = probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist);

                if (path.getLengthInMeter() != Double.MAX_VALUE) {
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            transitionLogProbability
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
                50.0
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }
}
