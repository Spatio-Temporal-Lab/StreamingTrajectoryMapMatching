package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.dmg.pmml.FieldName;
import org.dmg.pmml.PMML;
import org.jpmml.evaluator.*;
import org.jpmml.model.PMMLUtil;
import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
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
import java.io.*;
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

    private static final Evaluator evaluator;
    private List<Double> weights;
    private WindowBearing windowBearing = new WindowBearing();

    static {
        // 载入决策树模型
        try {
            String modelPath = "src/main/java/org/urbcomp/cupid/db/model/xgboost_weight_model.pmml";
            PMML pmml = PMMLUtil.unmarshal(Files.newInputStream(new File(modelPath).toPath()));
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

    public void loadWeight(int index) throws IOException {
        String weightPath =  "D:\\StreamingTrajectoryMapMatching\\weight.txt";
        BufferedReader br =new BufferedReader(new FileReader(weightPath));
        int i = 0;
        String line = null;
        while (i < index){
            i++;
            line = br.readLine();
        }
        String[] items = line.split(",");
        this.weights = Arrays.stream(items)
                .map(Double::parseDouble)
                .collect(Collectors.toList());
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

//                double treeProb = weights.get(index - 1);
//                double treeProb = getRectifyTransitionProbability(preTimeStep.getObservation(), timeStep.getObservation(), timeStep);
//                for (Map.Entry<CandidatePoint, Double> entry : timeStep.getEmissionLogProbabilities().entrySet()) {
//                    CandidatePoint key = entry.getKey();
//                    Double value = entry.getValue();
//                    timeStep.getEmissionLogProbabilities().put(key, value * treeProb);
//                }

                for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : timeStep.getTransitionLogProbabilities().entrySet()) {
                    Tuple2<CandidatePoint, CandidatePoint> key = entry.getKey();
                    Double value = entry.getValue();
                    double obBearing = windowBearing.getCurrBearing();
                    if (obBearing == -1){
                        continue;
                    }
                    RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                            key._1.getRoadSegmentId()
                    );
                    RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                            key._2.getRoadSegmentId()
                    );
                    Path subPath = paths.get(startRoadSegment.getEndNode())
                            .get(endRoadSegment.getStartNode());
                    Path path = pathAlgo.getCompletePath(key._1, key._2, subPath);
                    double canBearing = path.calDisWeightDirection(roadNetwork);
                    System.out.println("index:" + index + " key:" + key  + " " + canBearing);
                    double angleDifference = Math.abs(obBearing - canBearing);
                    if (angleDifference > 180) {
                        angleDifference = 360 - angleDifference;
                    }
                    double diff = 1 - angleDifference / 360.0;
                    timeStep.getTransitionLogProbabilities().put(key, value + Math.log(diff));
//                    double treeProb2 = getRectifyTransitionProbability(preTimeStep.getObservation(), timeStep.getObservation(), timeStep, preTimeStep, key);
//                    System.out.println(treeProb2);
//                    timeStep.getTransitionLogProbabilities().put(key, value*(1-treeProb));
                }
                System.out.println("index:" + index + "total: " +  windowBearing.getCurrBearing());

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
                seq.add(viterbi.computeMostLikelySequence().get(viterbi.computeMostLikelySequence().size()-1));
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            }else {
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
        if (StreamMapMatcher.findMaxValuePoint(viterbi.message) == null){
            return;
        }
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
                        double time = (curTimeStep.getObservation().getTime().getTime() - preTimeStep.getObservation().getTime().getTime()) * 1.0 / 1000;
                        double speed = path.getLengthInMeter() / time;
                        if (speed > 34) {
                            double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandiPt, curCandiPt);
                            if (disBtwCurAndPer < 34 * time) {
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
        if (map==null){
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


    private static Double getRectifyTransitionProbability(GPSPoint prePoint, GPSPoint currPoint, TimeStep currTimeStep)
            throws IOException, JAXBException, SAXException {
        Map<String, Double> featuresMap = new LinkedHashMap<>();

        double sum = 0;
        List<Double> allDistance = new ArrayList<>();
        for (CandidatePoint p : currTimeStep.getCandidates()) {
            double dis = GeoFunctions.getDistanceInM(p.getLng(), p.getLat(), currPoint.getLng(), currPoint.getLat());
            allDistance.add(dis);
            sum += dis;
        }

        featuresMap.put("avgDistance",  sum / allDistance.size());
        featuresMap.put("roadNum", (double) currTimeStep.getCandidates().size());
        featuresMap.put("distanceBtwPreAndCurr", GeoFunctions.getDistanceInM(prePoint.getLng(),prePoint.getLat(),currPoint.getLng(),currPoint.getLat()));
        featuresMap.put("timeBtwPreAndCurr", (double) ((currPoint.getTime().getTime() - prePoint.getTime().getTime())/1000));
        featuresMap.put("bearing", GeoFunctions.getBearing(prePoint.getLng(), prePoint.getLat(), currPoint.getLng(), currPoint.getLat()));

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
            primitiveValue = (float) computable.getResult();
        }
        return primitiveValue < 0 ? 0 : primitiveValue;
    }


    public static Double getRectifyTransitionProbability(GPSPoint prePoint, GPSPoint currPoint, TimeStep currTimeStep, TimeStep preTimeStep, Tuple2<CandidatePoint, CandidatePoint> tuple2)
            throws IOException, JAXBException, SAXException {
        //weight,avgDisBtwPreAndCurr,disBtwPreAndCurr,timeBtwPreAndCurr,roadNum,avgDistanceToRoad,distanceToRoad
        Map<String, Double> featuresMap = new LinkedHashMap<>();

        double sumToRoad = 0.0;
        List<Double> allDistanceToRoad = new ArrayList<>();
        for (CandidatePoint p : currTimeStep.getCandidates()) {
            double dis = GeoFunctions.getDistanceInM(p.getLng(), p.getLat(), currPoint.getLng(), currPoint.getLat());
            allDistanceToRoad.add(dis);
            sumToRoad += dis;
        }
        sumToRoad = sumToRoad / allDistanceToRoad.size();

        double sumBtwPreAndCurr = 0.0;
        List<Double> allDistanceBtwPreAndCurr = new ArrayList<>();
        for (CandidatePoint p1 : currTimeStep.getCandidates()) {
            for (CandidatePoint p2 : preTimeStep.getCandidates()){
                double distance = GeoFunctions.getDistanceInM(p1.getLng(), p1.getLat(), p2.getLng(), p2.getLat());
                sumBtwPreAndCurr += distance;
                allDistanceBtwPreAndCurr.add(distance);
            }
        }
        sumBtwPreAndCurr = sumBtwPreAndCurr/allDistanceBtwPreAndCurr.size();

        double obBearing = GeoFunctions.getBearing(currPoint.getLng(), currPoint.getLat(), prePoint.getLng(), prePoint.getLat());
        double candidateBearing = GeoFunctions.getBearing(tuple2._1.getLng(), tuple2._1.getLat(), tuple2._2.getLng(), tuple2._2.getLat());

        featuresMap.put("avgDisBtwPreAndCurr", sumBtwPreAndCurr);
        featuresMap.put("disBtwPreAndCurr", Math.abs(sumBtwPreAndCurr - GeoFunctions.getDistanceInM(tuple2._1.getLng(), tuple2._1.getLat(), tuple2._2.getLng(), tuple2._2.getLat())));
        featuresMap.put("timeBtwPreAndCurr", (currPoint.getTime().getTime() - prePoint.getTime().getTime()) / 1000.0);
        featuresMap.put("roadNum", currTimeStep.getCandidates().size() * 1.0);
        featuresMap.put("avgDistanceToRoad", sumToRoad);
        featuresMap.put("distanceToRoad", Math.abs(sumToRoad - GeoFunctions.getDistanceInM(tuple2._2.getLng(), tuple2._2.getLat(), currPoint.getLng(), currPoint.getLat())));
        featuresMap.put("bearing", Math.abs(obBearing - candidateBearing));

        Map<FieldName, FieldValue> arguments = new LinkedHashMap<>();
        for (InputField inputField : evaluator.getInputFields()) {
            FieldName inputFieldName = inputField.getName();
            Object rawValue = featuresMap.get(inputFieldName.getValue());
            FieldValue inputFieldValue = inputField.prepare(rawValue);
            arguments.put(inputFieldName, inputFieldValue);
        }

        Map<FieldName, ?> results = evaluator.evaluate(arguments);

        double score = 0;
        for (Map.Entry<FieldName, ?> entry : results.entrySet()) {
            Object result = entry.getValue();
            if (result instanceof ProbabilityDistribution) {
                ProbabilityDistribution<?> distribution = (ProbabilityDistribution<?>) result;
                score = distribution.getProbability("1");
            }
        }
        return score;
    }

}
