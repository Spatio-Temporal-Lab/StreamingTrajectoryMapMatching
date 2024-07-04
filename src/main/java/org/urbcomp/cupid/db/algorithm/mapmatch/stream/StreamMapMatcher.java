package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.dmg.pmml.PMML;
import org.jpmml.evaluator.*;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
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
import scala.Tuple3;


import org.dmg.pmml.FieldName;

import javax.xml.bind.JAXBException;
import javax.xml.parsers.ParserConfigurationException;
import java.io.*;
import java.nio.file.Files;
import java.sql.Timestamp;
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

    // 声明静态变量
    private static final Evaluator evaluator;

    static {
        // 载入决策树模型
        try {
            String modelPath = "src/main/java/org/urbcomp/cupid/db/model/decision_tree_model.pmml";
            PMML pmml = org.jpmml.model.PMMLUtil.unmarshal(Files.newInputStream(new File(modelPath).toPath()));
            ModelEvaluatorFactory modelEvaluatorFactory = ModelEvaluatorFactory.newInstance();
            evaluator = modelEvaluatorFactory.newModelEvaluator(pmml);
        } catch (IOException | JAXBException | SAXException e) {
            throw new RuntimeException("Error loading PMML model", e);
        }
    }

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public MapMatchedTrajectory streamMapMatch(Trajectory traj, boolean useRectify) {
        TimeStep preTimeStep = null;
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        int index = 0;
        for (GPSPoint p : traj.getGPSPointList()) {
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2 = this.computeViterbiSequence(p, seq, index, preTimeStep, viterbi, useRectify);
            seq = tuple2._1();
            preTimeStep = tuple2._2();
            viterbi = tuple2._3();
            index++;
        }
//        if (seq.size() < size) { //添加最后的
//            seq.addAll(viterbi.computeMostLikelySequence());
//        }
        System.out.println("seq size: " + seq.size());
        System.out.println("traj size: " + traj.getGPSPointList().size());
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
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(GPSPoint point, List<SequenceState> seq, int index, TimeStep preTimeStep, TiViterbi viterbi, boolean useRectify) {
        TimeStep timeStep = this.createTimeStep(point);//轨迹点+候选点集
        if (timeStep == null) {
//            seq.addAll(viterbi.computeMostLikelySequence()); //计算之前最有可能的序列
            seq.add(new SequenceState(null, point)); //添加新状态
            viterbi = new TiViterbi();
            preTimeStep = null;
        } else {
            this.computeEmissionProbabilities(timeStep, probabilities);//计算观测概率
            if (preTimeStep == null) { //第一个点初始化概率
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            } else {//计算转移概率
                int maxProbPointRSId = StreamMapMatcher.findMaxValuePoint(viterbi.message).getRoadSegmentId();//找到最大概率的候选点
                this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, maxProbPointRSId, useRectify);
                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities()
                );//计算维特比
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
            seq.add(new SequenceState(maxPoint, point));
            preTimeStep = timeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
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
            int maxProbPointRSId,
            boolean useRectify
    ) {
        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        ); //观测点的距离

        Set<CandidatePoint> startPoints = new HashSet<>(prevTimeStep.getCandidates());
        Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());
        Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                startPoints,
                endPoints
        ); // 找最短路径

        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            int preRoadSegmentId = preCandiPt.getRoadSegmentId();
            Timestamp transitionTime = prevTimeStep.getObservation().getTime();
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(preRoadSegmentId);
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                int currRoadSegmentId = curCandiPt.getRoadSegmentId();
                boolean isSameRoad = currRoadSegmentId == preRoadSegmentId;
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(currRoadSegmentId);
                Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
                Path path = pathAlgo.getCompletePath(preCandiPt, curCandiPt, subPath);
//                System.out.println("preCandiPt:"+ preCandiPt +"\n" +"curCandiPt:"+ curCandiPt +"\n" + "path:" + path +"\n" + "path.getLengthInMeter():" + path.getLengthInMeter());
//                System.out.println(timeStep.getObservation().getTime().getTime() - prevTimeStep.getObservation().getTime().getTime());
                double speed = path.getLengthInMeter() * 1000 / (timeStep.getObservation().getTime().getTime() - transitionTime.getTime());
                double rectifyProbability = 0;
                if (useRectify && !isSameRoad) {
                    try {
                        rectifyProbability = Math.log(getRectifyTransitionProbability(preRoadSegmentId, currRoadSegmentId, transitionTime));
//                    rectifyProbability = getRectifyTransitionProbability(preRoadSegmentId, currRoadSegmentId, transitionTime);
                    } catch (IOException | SAXException | JAXBException e) {
                        throw new RuntimeException(e);
                    }
                }
                if (speed > 55.6 && currRoadSegmentId == preRoadSegmentId && currRoadSegmentId == maxProbPointRSId) {
//                    System.out.println("speed is too fast at index ");
//                    System.out.println("" + maxProbPointRSId + " " + curCandiPt.getRoadSegmentId() + " " + preCandiPt.getRoadSegmentId());
                    double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
//                    System.out.println("disBtwCurAndPer:" + disBtwCurAndPer);
                    if (disBtwCurAndPer < 20) {
                        double transitionProbability = probabilities.transitionLogProbability(0.0, 0.0);
                        if (useRectify) transitionProbability = transitionProbability + rectifyProbability;
                        timeStep.addTransitionLogProbability(preCandiPt, curCandiPt, transitionProbability);
                    }
                    continue;
                }
                if (path.getLengthInMeter() != Double.MAX_VALUE) {
                    double transitionProbability = probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist);
                    if (useRectify) transitionProbability = transitionProbability + rectifyProbability;
                    timeStep.addTransitionLogProbability(preCandiPt, curCandiPt, transitionProbability);
                }
            }
        }
    }

    private static Double getRectifyTransitionProbability(int preRoadSegmentId, int currRoadSegmentId, Timestamp transitionTime)
            throws IOException, JAXBException, SAXException {
        Calendar calendar = Calendar.getInstance();
        calendar.setTimeInMillis(transitionTime.getTime());
        Map<String, Integer> featuresMap = new LinkedHashMap<>();
        featuresMap.put("currentRoadSegmentId", preRoadSegmentId);
        featuresMap.put("nextRoadSegmentId", currRoadSegmentId);
        featuresMap.put("hour", calendar.get(Calendar.HOUR_OF_DAY));
        featuresMap.put("dayofweek", calendar.get(Calendar.DAY_OF_WEEK));

        Map<FieldName, FieldValue> arguments = new LinkedHashMap<>();
        for (InputField inputField : evaluator.getInputFields()) {
            FieldName inputFieldName = inputField.getName();
            Object rawValue = featuresMap.get(inputFieldName.getValue());
            FieldValue inputFieldValue = inputField.prepare(rawValue);
            arguments.put(inputFieldName, inputFieldValue);
        }

        Map<FieldName, ?> results = evaluator.evaluate(arguments);
        List<TargetField> targetFields = evaluator.getTargetFields();

        TargetField targetField = targetFields.get(0);
        FieldName targetFieldName = targetField.getName();

        Object targetFieldValue = results.get(targetFieldName);

        double primitiveValue = -1;
        if (targetFieldValue instanceof Computable) {
            Computable computable = (Computable) targetFieldValue;
//            System.out.println(computable.getResult());
            primitiveValue = (double) computable.getResult();
        }
        return primitiveValue;
    }

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
