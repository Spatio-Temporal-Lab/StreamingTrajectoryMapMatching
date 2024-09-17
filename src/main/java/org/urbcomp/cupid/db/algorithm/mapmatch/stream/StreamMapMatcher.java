package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import com.mxgraph.layout.hierarchical.mxHierarchicalLayout;
import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxCellRenderer;
import com.mxgraph.view.mxGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner.PathCache;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner.ProbabilitySum;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.GeoFunctions;
import scala.Tuple2;
import scala.Tuple3;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
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

    private static final Map<String, Double> hisProb = new HashMap<>();

    private PathCache pathCache = new PathCache();

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    static {
        try (BufferedReader br = new BufferedReader(new FileReader(
                "src/main/resources/data/history-prob/prob_every_time.csv"))) {
            br.readLine();
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                String key = parts[2] + "," + parts[3] + "," + parts[6] + "," + parts[7]; // 构建键
                Double probability = Double.parseDouble(parts[4]); // 解析概率
                hisProb.put(key, probability); // 存储在Map中
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj, double beta) throws AlgorithmExecuteException {

        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();

        Graph<String, DefaultWeightedEdge> graph = new DefaultDirectedWeightedGraph<>(DefaultWeightedEdge.class);
        Map<String, Double> nodeEmissionProb = new HashMap<>();
        Map<DefaultWeightedEdge, Double> edgeWeights = new HashMap<>();

        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);

            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeStep, viterbi, beta, graph, nodeEmissionProb, edgeWeights, i);
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
//        visualizeGraph(graph, edgeWeights, nodeEmissionProb);
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
                                                                                    Graph<String, DefaultWeightedEdge> graph,
                                                                                    Map<String, Double> nodeEmissionProb,
                                                                                    Map<DefaultWeightedEdge, Double> edgeWeights,
                                                                                    int gpsIndex)
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

//                Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
//                        startPoints,
//                        endPoints
//                );

                Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                        startPoints,
                        endPoints,
                        pathCache
                );

                // 处理观测点向后偏移
//                processBackward(preTimeStep, timeStep, viterbi, paths);

//                CandidatePoint preCandiPt = findMaxValuePoint(viterbi.message);
                // 计算观测概率
//                this.computeEmissionProbabilities(preCandiPt, timeStep, probabilities);

                // 计算观测概率
                this.computeEmissionProbabilities(timeStep, probabilities);

                // 计算转移概率
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                // 计算维特比
                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities(),
                        beta
                );

//                addNodesAndEdgesToGraph(graph, edgeWeights, nodeEmissionProb, preTimeStep, timeStep, gpsIndex);

            } else {
                //第一个点初始化概率
                this.computeEmissionProbabilities(timeStep, probabilities);//计算观测概率
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
        // 上一个最大概率路段 id
        int roadSegmentId = StreamMapMatcher.findMaxValuePoint(viterbi.message).getRoadSegmentId();

        // 当前候选路段是否包含上一个最大概率路段 id, 并且记录该路段对应的候选点
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
                    // 如果上一个候选点和当前候选点不同（在同一路段）
                    if (preCandiPt != curCandiPt) {
                        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                                curCandiPt.getRoadSegmentId()
                        );
                        Path subPath = paths.get(startRoadSegment.getEndNode())
                                .get(endRoadSegment.getStartNode());
                        Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
                        double speed = path.getLengthInMeter() * 1000 / (curTimeStep.getObservation().getTime().getTime() - preTimeStep.getObservation().getTime().getTime());
                        // 如果速度大于 34 且相邻候选点的距离小于 20，则将上一个候选h点添加到当前候选点中
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
     * 根据time step, previous time step和概率分布函数计算emission P
     *
     * @param preCandiPt  上一个timeStep概率最大的候选点
     * @param timeStep    timeStep
     * @param probability 建立好的概率分布函数
     */
    private void computeEmissionProbabilities(CandidatePoint preCandiPt, TimeStep timeStep, HmmProbabilities probability) {
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            final double dist = candiPt.getErrorDistanceInMeter();
            double emissionLogProb = probability.emissionLogProbability(dist);
            emissionLogProb = correctEmissionProbability(emissionLogProb, preCandiPt, candiPt);
            timeStep.addEmissionLogProbability(candiPt, emissionLogProb);
        }
    }

    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths
    ) {
        // 观测点的距离
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );

        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(preCandiPt.getRoadSegmentId());
            boolean needCorrect = true;

            // 初步判定是否需要修正
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());
                if (endRoadSegment.equals(startRoadSegment) || startRoadSegment.isEndPoint(curCandiPt)) {
//                    System.out.println("don't need correct:");
//                    System.out.println("end road segment: " + endRoadSegment);
//                    System.out.println("start road segment: " + startRoadSegment);
//                    System.out.println(endRoadSegment.equals(startRoadSegment));
//                    System.out.println("end node of start road segment: " + startRoadSegment.getEndNode());
//                    System.out.println("curr candidate point: " + curCandiPt);
//                    System.out.println(startRoadSegment.isEndPoint(curCandiPt));
                    needCorrect = false;
                }

                if (preCandiPt == curCandiPt) {
//                    System.out.println("don't need correct:");
//                    System.out.println("previous candidate point equals current candidate point!");
                    needCorrect = false;
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            probabilities.transitionLogProbability(0.0, 0.0)
                    );
                    break;
                }
            }

//            needCorrect = false;

            if (needCorrect) {
                correctTransitionProbabilities(preCandiPt, prevTimeStep, timeStep, probabilities, paths, linearDist, startRoadSegment);
            } else {
                simpleTransitionProbabilities(preCandiPt, timeStep, probabilities, paths, linearDist, startRoadSegment);
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

    private double correctEmissionProbability(double emissionProbability, CandidatePoint preCandiPt, CandidatePoint curCandiPt) {
        double candiBearing = GeoFunctions.calculateBearing(preCandiPt, curCandiPt);
        double w1 = 1.0, w2 = 1.0;
//        System.out.println("modify before: " + emissionProbability);
        // 直线行驶
        if (candiBearing <= 10 || candiBearing >= 350) {
//            System.out.println("preCandiPt: " + preCandiPt.getLat() + "," + preCandiPt.getLng());
//            System.out.println("curCandiPt: " + curCandiPt.getLat() + "," + curCandiPt.getLng());
            return emissionProbability;
        }

        RoadSegment roadSegment1 = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());
        RoadSegment roadSegment2 = roadNetwork.getRoadSegmentById(preCandiPt.getRoadSegmentId());
        double segmentBearing1 = roadSegment1.getBearing();
        double segmentBearing2 = roadSegment2.getBearing();
        double segmentBearing = (segmentBearing1 + segmentBearing2) / 2;

        double alpha = Math.abs(candiBearing - segmentBearing);
        alpha = Math.min(alpha, 360 - alpha);
        double directionProbability = (1 + Math.cos(Math.toRadians(alpha))) / 2;

//        System.out.println("modify after: " + (emissionProbability + Math.log(directionProbability)));
        return w1 * emissionProbability + w2 * Math.log(directionProbability);
    }

    private void correctTransitionProbabilities(
            CandidatePoint preCandiPt,
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths,
            double linearDist,
            RoadSegment startRoadSegment
    ) {
        Map<CandidatePoint, Double> transitionProbTobeCorrected = new HashMap<>();
        Map<CandidatePoint, Double> hisProbTobeCorrected = new HashMap<>();

        for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
            RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());
            Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
            Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);

            if (path.getLengthInMeter() != Double.MAX_VALUE) {
                double transitionProb = probabilities.transitionProbability(path.getLengthInMeter(), linearDist);
                transitionProbTobeCorrected.put(curCandiPt, transitionProb);

                String key = getKey(prevTimeStep, preCandiPt, curCandiPt);
                if (hisProb.containsKey(key)) {
                    hisProbTobeCorrected.put(curCandiPt, hisProb.get(key));
                }
            }
        }

        ProbabilitySum probabilitySum = ProbabilitySum.calculateSum(transitionProbTobeCorrected, hisProbTobeCorrected);
        double transitionSum = probabilitySum.getTransitionSum();
        double historySum = probabilitySum.getHistorySum();

//        double transitionSum = calculateSum(transitionProbTobeCorrected, hisProbTobeCorrected);
//        double historySum = hisProbTobeCorrected.values().stream().mapToDouble(Double::doubleValue).sum();

        applyCorrectedProbabilities(preCandiPt, timeStep, transitionProbTobeCorrected, hisProbTobeCorrected, transitionSum, historySum);
    }

    private void simpleTransitionProbabilities(
            CandidatePoint preCandiPt,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths,
            double linearDist,
            RoadSegment startRoadSegment
    ) {
        for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
            RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());
            Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
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

    private double calculateSum(Map<CandidatePoint, Double> transitionProbTobeCorrected, Map<CandidatePoint, Double> hisProbTobeCorrected) {
        double transitionSum = 0;
        for (CandidatePoint candi : hisProbTobeCorrected.keySet()) {
            // 负无穷大的概率不计入
            if (transitionProbTobeCorrected.containsKey(candi) &&
                    transitionProbTobeCorrected.get(candi) != Double.NEGATIVE_INFINITY) {
                transitionSum += transitionProbTobeCorrected.get(candi);
            }
        }
        return transitionSum;
    }

    private void applyCorrectedProbabilities(
            CandidatePoint preCandiPt,
            TimeStep timeStep,
            Map<CandidatePoint, Double> transitionProbTobeCorrected,
            Map<CandidatePoint, Double> hisProbTobeCorrected,
            double transitionSum,
            double historySum
    ) {
        for (CandidatePoint curCandiPt : transitionProbTobeCorrected.keySet()) {
            double correctedProb = transitionProbTobeCorrected.get(curCandiPt);
            if (correctedProb == Double.NEGATIVE_INFINITY) {
                timeStep.addTransitionLogProbability(
                        preCandiPt,
                        curCandiPt,
                        correctedProb
                );
                continue;
            }
//            System.out.println("--------------------------------------------------");
//            System.out.println("start correct!");
//             && isAdjacent(preCandiPt, curCandiPt)
            if (hisProbTobeCorrected.containsKey(curCandiPt)) {
//                System.out.println("correct before: " + Math.log(correctedProb));
                double scale = transitionSum / historySum;
                double historyProbability = hisProbTobeCorrected.get(curCandiPt);
//                System.out.println("transition sum: " + transitionSum);
//                System.out.println("history sum: " + historySum);
//                System.out.println("history prob: " + historyProbability);
//                System.out.println("transition prob: " + correctedProb);
//                correctedProb = Math.log(0.5 * (transitionSum * (historyProbability / historySum) + correctedProb));
//                correctedProb = Math.log(transitionSum * (historyProbability / historySum) + correctedProb);
//                correctedProb = Math.log(transitionSum * (historyProbability / historySum));
//                correctedProb = Math.log(transitionSum * (historyProbability / historySum)) + Math.log(correctedProb);
//                correctedProb = Math.log(scale * (transitionSum * (historyProbability / historySum) + correctedProb));
                correctedProb = Math.log(correctedProb) + Math.log(1 + historyProbability);
//                System.out.println("correct after: " + correctedProb);
            } else correctedProb = Math.log(correctedProb);
            timeStep.addTransitionLogProbability(
                    preCandiPt,
                    curCandiPt,
                    correctedProb
            );
        }

        transitionProbTobeCorrected.clear();
        hisProbTobeCorrected.clear();
    }

    private String getKey(TimeStep prevTimeStep, CandidatePoint preCandiPt, CandidatePoint curCandiPt) {
        Calendar calendar = Calendar.getInstance();
        calendar.setTime(prevTimeStep.getObservation().getTime());
        int hour = calendar.get(Calendar.HOUR_OF_DAY);
        int dayOfWeek = calendar.get(Calendar.DAY_OF_WEEK);
        return hour + "," + dayOfWeek + "," + preCandiPt.getRoadSegmentId() + "," + curCandiPt.getRoadSegmentId();
    }

    private boolean isAdjacent(CandidatePoint preCandiPt, CandidatePoint curCandiPt) {
        RoadNode r1 = roadNetwork.getRoadSegmentById(preCandiPt.getRoadSegmentId()).getEndNode();
        RoadNode r2 = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId()).getStartNode();
        return (r1.getLat() == r2.getLat()) && (r1.getLng() == r2.getLng());
    }

    private void visualizeGraph(Graph<String, DefaultWeightedEdge> graph, Map<DefaultWeightedEdge, Double> edgeWeights, Map<String, Double> nodeEmissionProb) {
        mxGraph jGraph = new mxGraph();
        Object parent = jGraph.getDefaultParent();

        jGraph.getModel().beginUpdate();
        try {
            Map<String, Object> vertices = new HashMap<>();
            for (String vertex : graph.vertexSet()) {
                Double emissionProb = nodeEmissionProb.get(vertex);
                String vertexLabel = vertex + "\nE: " + (emissionProb != null ? emissionProb : "N/A");
                vertices.put(vertex, jGraph.insertVertex(parent, null, vertexLabel, 0, 0, 80, 30)); // Adjust initial position and size
            }
            for (DefaultWeightedEdge edge : graph.edgeSet()) {
                String source = graph.getEdgeSource(edge);
                String target = graph.getEdgeTarget(edge);
                Double weight = edgeWeights.get(edge);
                jGraph.insertEdge(parent, null, weight, vertices.get(source), vertices.get(target));
            }
        } finally {
            jGraph.getModel().endUpdate();
        }

        // Apply layout for better spacing
        mxHierarchicalLayout layout = new mxHierarchicalLayout(jGraph);
        layout.setIntraCellSpacing(100); // Increase spacing between nodes
        layout.execute(parent);

        // Save the graph to a file
        saveGraphToFile(jGraph, "graph.png");

        // Display the graph in a JFrame
        mxGraphComponent graphComponent = new mxGraphComponent(jGraph);
        JFrame frame = new JFrame("Graph Visualization");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(graphComponent);
        frame.setSize(1200, 800); // Set a larger size for the window
        frame.setVisible(true);
    }


    private void saveGraphToFile(mxGraph jGraph, String filename) {
        try {
            BufferedImage image = mxCellRenderer.createBufferedImage(jGraph, null, 2, java.awt.Color.WHITE, true, null);
            File imgFile = new File(filename);
            ImageIO.write(image, "PNG", imgFile);
            System.out.println("Graph saved to " + imgFile.getAbsolutePath());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    private void addNodesAndEdgesToGraph(
            Graph<String, DefaultWeightedEdge> graph,
            Map<DefaultWeightedEdge, Double> edgeWeights,
            Map<String, Double> nodeEmissionProb,
            TimeStep preTimeStep,
            TimeStep timeStep,
            int gpsIndex
    ) {
        for (CandidatePoint preCandi : preTimeStep.getCandidates()) {
            Map<CandidatePoint, Double> preEmissionProb = preTimeStep.getEmissionLogProbabilities();
            Map<CandidatePoint, Double> curEmissionProb = timeStep.getEmissionLogProbabilities();
            String preNode = "_rid" + preCandi.getRoadSegmentId() + "_index" + preCandi.getMatchedIndex() + "_gps" + (gpsIndex - 1);
            graph.addVertex(preNode);
            nodeEmissionProb.put(preNode, preEmissionProb.get(preCandi));

            for (CandidatePoint curCandi : timeStep.getCandidates()) {
                String curNode = "_rid" + curCandi.getRoadSegmentId() + "_index" + curCandi.getMatchedIndex() + "_gps" + gpsIndex;
                graph.addVertex(curNode);
                nodeEmissionProb.put(curNode, curEmissionProb.get(curCandi));

                double transitionProb = timeStep.getTransitionLogProbabilities().get(Tuple2.apply(preCandi, curCandi));

                if (transitionProb != Double.NEGATIVE_INFINITY) {
                    DefaultWeightedEdge edge = graph.addEdge(preNode, curNode);
                    graph.setEdgeWeight(edge, transitionProb);
                    edgeWeights.put(edge, transitionProb);
                }
            }
        }
    }

    public static void main(String[] args) throws AlgorithmExecuteException {
        Trajectory trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        StreamMapMatcher mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher2.streamMapMatch(trajectory, 0.5);
    }
}
