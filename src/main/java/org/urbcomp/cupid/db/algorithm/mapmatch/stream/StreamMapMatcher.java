package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.history.generateHistoryProb;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner.PathCache;
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
import org.xml.sax.SAXException;
import scala.Tuple2;
import scala.Tuple3;

import javax.xml.bind.JAXBException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.util.*;
import java.util.stream.Collectors;

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
    protected generateHistoryProb historyProb = null;

    private WindowBearing windowBearing = new WindowBearing();

    private PathCache pathCache = new PathCache();

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo, generateHistoryProb historyProb) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
        this.historyProb = historyProb;
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

                // 加速路径计算
                Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(startPoints, endPoints, pathCache);
//                Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
//                        startPoints,
//                        endPoints
//                );

                // 处理观测点向后偏移
                this.processBackward(preTimeStep, timeStep, viterbi, paths);

                //计算观测概率
                this.computeEmissionProbabilities(timeStep, probabilities);

                //计算转移概率
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                //计算历史概率
//                this.adjustTransitionProbabilities(timeStep, StreamMapMatcher.findMaxValuePoint(viterbi.message));

                //根据方向修正
//                this.adjustWithDirection(timeStep, preTimeStep, paths, probabilities);

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
//        System.out.println(viterbi.message);
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
//                System.out.println(curTimeStep.getObservation() + " " + path.getRoadSegmentIds() + " " +path.getPoints() + " " + speed + " " + path.getLengthInMeter());
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
//        if (StreamMapMatcher.findMaxValuePoint(viterbi.message) == null){
//            return;
//        }
//        int roadSegmentId = StreamMapMatcher.findMaxValuePoint(viterbi.message).getRoadSegmentId();
//        List<CandidatePoint> curCandidates = curTimeStep.getCandidates();
//        int i = 0;
//        for (; i < curCandidates.size(); i++) {
//            if (curCandidates.get(i).getRoadSegmentId() == roadSegmentId) {
//                break;
//            }
//        }
//        if (i != curCandidates.size()) {
//            for (CandidatePoint preCandiPt : preTimeStep.getCandidates()) {
//                if (preCandiPt.getRoadSegmentId() == roadSegmentId) {
//
//                    RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
//                            preCandiPt.getRoadSegmentId()
//                    );
//                    CandidatePoint curCandiPt = curCandidates.get(i);
//                    if (preCandiPt != curCandiPt) {
//                        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
//                                curCandiPt.getRoadSegmentId()
//                        );
//                        Path subPath = paths.get(startRoadSegment.getEndNode())
//                                .get(endRoadSegment.getStartNode());
//                        Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
//                        double time = (curTimeStep.getObservation().getTime().getTime() - preTimeStep.getObservation().getTime().getTime()) * 1.0 / 1000;
//                        double speed = path.getLengthInMeter() / time;
//                        if (speed > 34) {
//                            double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
//                            if (disBtwCurAndPer < 34 * time) {
//                                curTimeStep.addCandidate(preCandiPt);
//                            }
//                        }
//                    }
//                }
//            }
//        }
    }

    public void adjustTransitionProbabilities(TimeStep timeStep, CandidatePoint preSelectPoint) {
        CandidatePoint currEqualPre = null;
        // 查找当前等于先前选择的候选点
        for (CandidatePoint candidatePoint : timeStep.getCandidates()) {
            if (candidatePoint.getRoadSegmentId() == preSelectPoint.getRoadSegmentId()) {
                currEqualPre = candidatePoint;
            }
        }
        // 如果找到了匹配的候选点，且其不是先前候选点的结束节点
        if (currEqualPre != null && !roadNetwork.getRoadSegmentById(preSelectPoint.getRoadSegmentId()).getEndNode().equals(currEqualPre)) {
            return;
        }
        Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities = timeStep.getTransitionLogProbabilities();
        Map<Integer, Double> totalTransProbPreRoad = new HashMap<>();
        int week = timeStep.getObservation().getTime().toLocalDateTime().getDayOfWeek().getValue();

        Map<Integer, Double> temp = new HashMap<>();
        Map<Integer, Double> temp2 = new HashMap<>();

        // 添加概率
        for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : transitionLogProbabilities.entrySet()) {
            int startId = entry.getKey()._1.getRoadSegmentId();
            int endId = entry.getKey()._2.getRoadSegmentId();
            // 如果起始道路段的结束节点等于结束道路段的起始节点且起始 ID 等于 preSelectPoint 的 ID
            if (roadNetwork.getRoadSegmentById(startId).getEndNode().equals(roadNetwork.getRoadSegmentById(endId).getStartNode()) && startId == preSelectPoint.getRoadSegmentId()) {
                totalTransProbPreRoad.put(startId, totalTransProbPreRoad.getOrDefault(startId, 0.0) + Math.exp(entry.getValue()) * historyProb.getHisProb(startId, endId, week));
                temp.put(endId, Math.exp(entry.getValue()));
                temp2.put(endId, historyProb.getHisProb(startId, endId, week));
            }
        }
        System.out.println(totalTransProbPreRoad);
        System.out.println(temp);
        System.out.println(temp2);
        System.out.println("#################");
        double totalFixProb = 0.0;
        double totalPreProb = 0.0;

        // 统计基础概率
        for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : transitionLogProbabilities.entrySet()) {
            int startId = entry.getKey()._1.getRoadSegmentId();
            int endId = entry.getKey()._2.getRoadSegmentId();
            if (roadNetwork.getRoadSegmentById(startId).getEndNode().equals(roadNetwork.getRoadSegmentById(endId).getStartNode()) && startId == preSelectPoint.getRoadSegmentId()) {
                double nowProb = Math.exp(transitionLogProbabilities.get(entry.getKey()));
                double fixProb = nowProb * historyProb.getHisProb(startId, endId, week) / totalTransProbPreRoad.get(startId);
                totalFixProb += fixProb;
                totalPreProb += nowProb;
            }
        }

        // 在基础概率的基础上计算概率
        for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : transitionLogProbabilities.entrySet()) {
            int startId = entry.getKey()._1.getRoadSegmentId();
            int endId = entry.getKey()._2.getRoadSegmentId();
            if (roadNetwork.getRoadSegmentById(startId).getEndNode().equals(roadNetwork.getRoadSegmentById(endId).getStartNode()) && startId == preSelectPoint.getRoadSegmentId()) {
                double nowProb = Math.exp(transitionLogProbabilities.get(entry.getKey()));
                double fixProb = nowProb * historyProb.getHisProb(startId, endId, week) / totalTransProbPreRoad.get(startId);
                transitionLogProbabilities.put(entry.getKey(), Math.log(nowProb * 0.5 + fixProb * (totalPreProb / totalFixProb) * 0.5));
            }
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
            } else { // 速度大于 2.0
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
                    // 引入方向的转移概率
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

//                // 两个候选点相同，将概率置为极大
//                if (preCandiPt == curCandiPt) {
//                    timeStep.addTransitionLogProbability(
//                            preCandiPt,
//                            curCandiPt,
//                            probabilities.transitionLogProbability(0.0, 0.0)
//                    );
//                    continue;
//                }

                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                        curCandiPt.getRoadSegmentId()
                );
                Path subPath = paths.get(startRoadSegment.getEndNode())
                        .get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                double transitionLogProbability = probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist);

//                if (startRoadSegment.getRoadSegmentId() == 79145){
//                    System.out.println("ob:" + linearDist);
//                    System.out.println("sub: " + curCandiPt.getRoadSegmentId() + " " + subPath);
//                    System.out.println("all: " + curCandiPt.getRoadSegmentId() + " " + path);
//                }

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
