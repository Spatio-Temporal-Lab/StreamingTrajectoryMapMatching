package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

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

import java.io.BufferedReader;
import java.io.FileReader;
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
    private static final double transitionProbabilityBeta = 2;

    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    /**
     * 路网
     */
    protected final RoadNetwork roadNetwork;

    protected final AbstractManyToManyShortestPath pathAlgo;

    private static final Map<String, Double> historyProbabilities = new HashMap<>();

    private static final Map<CandidatePoint, Double> rectifyHistoryProbabilities = new HashMap<>();
    private static final Map<CandidatePoint, Double> rectifyTransitionProbabilities = new HashMap<>();

    static {
        // 静态代码块，在类加载时执行一次
        try (BufferedReader br = new BufferedReader(new FileReader("src/main/resources/data/history_probability/prob_every_time.csv"))) {
            br.readLine();
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                String key = parts[2] + "," + parts[3] + "," + parts[6] + "," + parts[7]; // 构建键
                Double probability = Double.parseDouble(parts[4]); // 解析概率
                historyProbabilities.put(key, probability); // 存储在Map中
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj, double beta, boolean useCorrect) throws AlgorithmExecuteException {

        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);

            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeStep, viterbi, beta, useCorrect);
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
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point,
                                                                                    List<SequenceState> seq,
                                                                                    TimeStep preTimeStep,
                                                                                    TiViterbi viterbi,
                                                                                    Double beta,
                                                                                    boolean useRectify)
            throws AlgorithmExecuteException {
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
                Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                        startPoints,
                        endPoints
                );

                // 处理观测点向后偏移
                processBackward(preTimeStep, timeStep, viterbi, paths);

                //计算观测概率
                this.computeEmissionProbabilities(preTimeStep, timeStep, probabilities);

                //计算转移概率
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths, useRectify);

                //计算维特比
                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities(),
                        beta
                );
            } else {
                //第一个点初始化概率
                this.computeEmissionProbabilities(timeStep, probabilities); // 计算观测概率
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        beta
                );
            }
            if (viterbi.isBroken) {
//                seq.addAll(viterbi.computeMostLikelySequence());
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        beta
                );
            }

            CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
            seq.add(new SequenceState(maxPoint, point));
            preTimeStep = timeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
    }

    // 处理观测点向后偏移的情况
    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep, TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths) {
        // 获取上一个时间步最大概率路径的道路段ID
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
                // 找到上一个时间步对应的最大概率候选点
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
                        // 如果十分接近，则以上一个最大概率点作为当前最大概率点
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
     * 根据time step, previous time step和概率分布函数计算emission P
     *
     * @param prevTimeStep prevTimeStep
     * @param timeStep     timeStep
     * @param probability  建立好的概率分布函数
     */
    private void computeEmissionProbabilities(TimeStep prevTimeStep, TimeStep timeStep, HmmProbabilities probability) {
        final double gpsBearing = GeoFunctions.calculateBearing(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            final double dist = candiPt.getErrorDistanceInMeter();
            double emissionLogProb = probability.emissionLogProbability(dist);
//            emissionLogProb = correctEmissionProbability(emissionLogProb, gpsBearing, candiPt);
            timeStep.addEmissionLogProbability(candiPt, emissionLogProb);
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
     * @param prevTimeStep  之前的 timestep
     * @param timeStep      当前的 timestep
     * @param probabilities 建立好的概率分布函数
     * @param paths         候选点间最短路径
     * @param useCorrect    是否使用历史概率修正
     */
    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths,
            boolean useCorrect
    ) throws AlgorithmExecuteException {
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );

        // 将转移概率添加到当前时间步
        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(preCandiPt.getRoadSegmentId());
            Calendar calendar = Calendar.getInstance();
            calendar.setTime(prevTimeStep.getObservation().getTime());

            boolean needCorrect = true;

            // 将转移概率添加到字典中
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {

                if (preCandiPt == curCandiPt) {
                    needCorrect = false;
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            probabilities.transitionLogProbability(0.0, 0.0)
                    );
                    continue;
                }

                storeProbabilityToMap(paths, preCandiPt, curCandiPt, linearDist, calendar);

                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());

                // 判断是否需要进行修正
                if (endRoadSegment.equals(startRoadSegment) && !startRoadSegment.isEndPoint(curCandiPt))
                    needCorrect = false;

            }

            // 如果需要修正，调用修正方法
            if (needCorrect && useCorrect) {
                correctTransitionProbabilities(preCandiPt, timeStep);
            } else {
                // 按照正常的概率计算
                for (CandidatePoint curCandiPt : rectifyTransitionProbabilities.keySet()) {
                    double transitionProbability = rectifyTransitionProbabilities.get(curCandiPt);
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            Math.log(transitionProbability)
                    );
                }
            }
        }
    }

    private void correctTransitionProbabilities(CandidatePoint preCandiPt, TimeStep timeStep) {
        double transitionSum = 0.0;
        for (CandidatePoint candi : rectifyHistoryProbabilities.keySet()) {
            if (rectifyTransitionProbabilities.containsKey(candi))
                transitionSum += rectifyTransitionProbabilities.get(candi);
        }
        double historySum = rectifyHistoryProbabilities.values().stream().mapToDouble(Double::doubleValue).sum();

        for (CandidatePoint curCandiPt : rectifyTransitionProbabilities.keySet()) {
            double transitionProbability = rectifyTransitionProbabilities.get(curCandiPt);
            if (rectifyHistoryProbabilities.containsKey(curCandiPt)) {
                double historyProbability = rectifyHistoryProbabilities.get(curCandiPt);
                transitionProbability = 0.5 * (transitionSum * (historyProbability / historySum) + transitionProbability);
            }

            timeStep.addTransitionLogProbability(
                    preCandiPt,
                    curCandiPt,
                    Math.log(transitionProbability)
            );
        }
        rectifyHistoryProbabilities.clear();
        rectifyTransitionProbabilities.clear();
    }

    private void storeProbabilityToMap(Map<RoadNode, Map<RoadNode, Path>> paths,
                                       CandidatePoint preCandiPt, CandidatePoint curCandiPt,
                                       double linearDist, Calendar calendar) {

        RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(preCandiPt.getRoadSegmentId());
        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());

        Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
        Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);

        boolean isAdjacent = endRoadSegment.getStartNode().equalsByGeometry(startRoadSegment.getEndNode());
        int hour = calendar.get(Calendar.HOUR_OF_DAY);
        int dayOfWeek = calendar.get(Calendar.DAY_OF_WEEK);

        if (path.getLengthInMeter() != Double.MAX_VALUE) {
            // 只修正邻接点
            if (isAdjacent) {
                String key = hour + "," + dayOfWeek + "," + preCandiPt.getRoadSegmentId() + "," + curCandiPt.getRoadSegmentId();
                // 需要存在历史概率
                if (historyProbabilities.containsKey(key)) {
                    double historyProbability = historyProbabilities.get(key);
                    rectifyHistoryProbabilities.put(curCandiPt, historyProbability);
                }
            }

            double transitionProbability = probabilities.transitionPositionProbability(path.getLengthInMeter(), linearDist);
            rectifyTransitionProbabilities.put(curCandiPt, transitionProbability);

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

    private double correctEmissionProbability(double emissionProbability, double gpsBearing, CandidatePoint candidatePoint) {
        RoadSegment roadSegment = roadNetwork.getRoadSegmentById(candidatePoint.getRoadSegmentId());
        double segmentBearing = GeoFunctions.calculateBearing(roadSegment.getStartNode(), roadSegment.getEndNode());
        double alpha = Math.abs(gpsBearing - segmentBearing);
        alpha = Math.min(alpha, 360 - alpha);
        double directionProbability = (1 + Math.cos(Math.toRadians(alpha))) / 2;
        return emissionProbability + Math.log(directionProbability);
    }

}
