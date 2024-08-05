package org.urbcomp.cupid.db.weight;

import org.geojson.Feature;
import org.geojson.Point;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.util.GeoFunctions;
import scala.Tuple2;

import java.util.*;
import java.util.stream.DoubleStream;

public class findWeight {

    private final RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();

    private static final double measurementErrorSigma = 50.0;
    private static final double transitionProbabilityBeta = 2;
    private final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    private final AbstractManyToManyShortestPath pathAlgo = new ManyToManyShortestPath(roadNetwork);

    public findWeight() {
    }

    public void getWeight(Feature prePointFeature, Feature currPointFeature, GPSPoint prePoint, GPSPoint currPoint, List<Double> weightForATra) {
        Point point = (Point) prePointFeature.getGeometry();
        CandidatePoint preSelectPoint = new CandidatePoint(new SpatialPoint(point.getCoordinates().getLongitude(), point.getCoordinates().getLatitude()), roadNetwork.getRoadSegmentById(prePointFeature.getProperty("roadSegmentId")), prePointFeature.getProperty("matchedIndex"), prePointFeature.getProperty("errorDistanceInMeter"));
        point = (Point) currPointFeature.getGeometry();
        CandidatePoint currSelectPoint = new CandidatePoint(new SpatialPoint(point.getCoordinates().getLongitude(), point.getCoordinates().getLatitude()), roadNetwork.getRoadSegmentById(currPointFeature.getProperty("roadSegmentId")), currPointFeature.getProperty("matchedIndex"), currPointFeature.getProperty("errorDistanceInMeter"));
        TimeStep preTimeStep = this.createTimeStep(prePoint);
        TimeStep currTimeStep = this.createTimeStep(currPoint);
        if (preTimeStep == null || currTimeStep == null) {
            weightForATra.add(0.5);
            return ;
        }
        Set<CandidatePoint> startPoints = new HashSet<>(preTimeStep.getCandidates());
        Set<CandidatePoint> endPoints = new HashSet<>(currTimeStep.getCandidates());
        if (!startPoints.contains(preSelectPoint) || !endPoints.contains(currSelectPoint)) {
            weightForATra.add(0.5);
            return;
        }
        this.computeEmissionProbabilities(currTimeStep, probabilities);
        Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                startPoints,
                endPoints
        );
        this.computeTransitionProbabilities(preTimeStep, currTimeStep, probabilities, paths);
        //add weight
        weightForATra.add(calBestWeight(preTimeStep, currTimeStep, currSelectPoint, preSelectPoint));
//        //add avgDistance
        weightForATra.add(getAvgDistance(currPoint));
//        //roadNum
        weightForATra.add(getRoadNum(currPoint));
//        //disBtwPreAndCurr
        weightForATra.add(getDisBtwPreAndCurr(prePoint, currPoint));
//        //timeBtwPreAndCurr
        weightForATra.add(getTimeBtwPreAndCurr(prePoint, currPoint));
//        //bearing
        weightForATra.add(GeoFunctions.getBearing(prePoint.getLng(), prePoint.getLat(), currPoint.getLng(), currPoint.getLat()));

    }

    private double calBestWeight(TimeStep preTimeStep, TimeStep currTimeStep, CandidatePoint currSelectPoint, CandidatePoint preSelectPoint) {
        int numSteps = 100;
        double[] kValues = DoubleStream.iterate(0, n -> n + 1.0 / numSteps)
                .limit(numSteps)
                .toArray();
        kValues = Arrays.copyOf(kValues, kValues.length + 1);
        kValues[kValues.length - 1] = 1.0;

        HashMap<Tuple2<CandidatePoint, CandidatePoint>, Double> allProbes;
        HashMap<Double, Double> bestKs = new HashMap<>();

        for (double kValue : kValues) {
            allProbes = new HashMap<>();
            calcProbabilities(preTimeStep, currTimeStep, kValue, allProbes, preSelectPoint);
            Tuple2<Boolean, Double> result = isLargest(currSelectPoint, allProbes);
            if (result._1) {
                bestKs.put(kValue, result._2);
            }
        }
        return findBestK(bestKs);
    }

    private double findBestK(HashMap<Double, Double> bestKs) {
        if (bestKs.isEmpty()) {
            return 0.5;
        } else {
            List<Double> bestKList = new ArrayList<>();
            for (Map.Entry<Double, Double> entry : bestKs.entrySet()) {
                bestKList.add(entry.getKey());
            }
            return findMaxK(bestKList);
        }
    }


    private double findMaxK(List<Double> bestKList) {
        Collections.sort(bestKList);
        System.out.println(bestKList);
        double sum = 0;
        for (Double k : bestKList) {
           sum += k;
        }
        return sum / bestKList.size();
    }


    private Tuple2<Boolean, Double> isLargest(CandidatePoint currSelectPoint, HashMap<Tuple2<CandidatePoint, CandidatePoint>, Double> allProbes) {
        boolean isMax = true;
        double max = Double.NEGATIVE_INFINITY;
        List<CandidatePoint> maxCandidatePoints = new ArrayList<>();
        for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry : allProbes.entrySet()) {
            Tuple2<CandidatePoint, CandidatePoint> tuple2 = entry.getKey();
            double currScores = entry.getValue();
            if (currScores > max){
                maxCandidatePoints.clear();
                max = currScores;
                maxCandidatePoints.add(tuple2._2);
            }else if(currScores == max && max!=Double.NEGATIVE_INFINITY){
                maxCandidatePoints.add(tuple2._2);
            }
        }
        if (!maxCandidatePoints.contains(currSelectPoint)){
            isMax = false;
        }
        return new Tuple2<>(isMax, max);
    }

    private void calcProbabilities(TimeStep preTimeStep, TimeStep currTimeStep, double k,  HashMap<Tuple2<CandidatePoint, CandidatePoint>, Double> allProb, CandidatePoint preSelectPoint) {
        List<CandidatePoint> currCandidatePoints = currTimeStep.getCandidates();
        List<CandidatePoint> preCandidatePoints = preTimeStep.getCandidates();

        for (CandidatePoint curr : currCandidatePoints){
            Tuple2<CandidatePoint, CandidatePoint> tuple2 = new Tuple2<>(preSelectPoint, curr);
            double score = currTimeStep.getEmissionLogProbabilities().get(curr) * k + currTimeStep.getTransitionLogProbabilities().get(tuple2) * (1 - k);
            if (Double.isNaN(score)) {
                score = Double.NEGATIVE_INFINITY;
            }
            allProb.put(tuple2, score);
        }


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

    private void computeEmissionProbabilities(TimeStep timeStep, HmmProbabilities probability) {
        for (CandidatePoint candiPt : timeStep.getCandidates()) {
            final double dist = candiPt.getErrorDistanceInMeter();
            timeStep.addEmissionLogProbability(candiPt, probability.emissionLogProbability(dist));
        }
    }

    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths
    ) {
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
                } else {
                    timeStep.addTransitionLogProbability(
                            preCandiPt,
                            curCandiPt,
                            Double.NEGATIVE_INFINITY
                    );
                }
            }
        }
    }

    public double getAvgDistance(GPSPoint currPoint) {
        TimeStep currTimeStep = this.createTimeStep(currPoint);
        if (currTimeStep == null) {
            return -1;
        }
        double sum = 0;
        List<Double> allDistance = new ArrayList<>();
        for (CandidatePoint p : currTimeStep.getCandidates()) {
            double dis = GeoFunctions.getDistanceInM(p.getLng(), p.getLat(), currPoint.getLng(), currPoint.getLat());
            allDistance.add(dis);
            sum += dis;
        }
        return sum / allDistance.size();
    }

    public double getRoadNum(GPSPoint currPoint) {
        TimeStep currTimeStep = this.createTimeStep(currPoint);
        if (currTimeStep == null) {
            return -1;
        }
        return currTimeStep.getCandidates().size();
    }

    public double getDisBtwPreAndCurr(GPSPoint prePoint, GPSPoint currPoint) {
        return GeoFunctions.getDistanceInM(prePoint.getLng(), prePoint.getLat(), currPoint.getLng(), currPoint.getLat());
    }

    public double getTimeBtwPreAndCurr(GPSPoint prePoint, GPSPoint currPoint) {
        return (currPoint.getTime().getTime() - prePoint.getTime().getTime()) / 1000.0;
    }


    //weight,avgDisBtwPreAndCurr,disBtwPreAndCurr,timeBtwPreAndCurr,roadNum,avgDistanceToRoad,distanceToRoad,bearing
    public void calAllFeatureForAPoint(List<List<Double>> allForAPoint, Feature prePointFeature, Feature currPointFeature, GPSPoint prePoint, GPSPoint currPoint) {
        Point point = (Point) currPointFeature.getGeometry();
        CandidatePoint currSelectPoint = new CandidatePoint(new SpatialPoint(point.getCoordinates().getLongitude(), point.getCoordinates().getLatitude()), roadNetwork.getRoadSegmentById(currPointFeature.getProperty("roadSegmentId")), currPointFeature.getProperty("matchedIndex"), currPointFeature.getProperty("errorDistanceInMeter"));
        Point point2 = (Point) prePointFeature.getGeometry();
        CandidatePoint preSelectPoint = new CandidatePoint(new SpatialPoint(point2.getCoordinates().getLongitude(), point2.getCoordinates().getLatitude()), roadNetwork.getRoadSegmentById(prePointFeature.getProperty("roadSegmentId")), prePointFeature.getProperty("matchedIndex"), prePointFeature.getProperty("errorDistanceInMeter"));

        TimeStep preTimeStep = this.createTimeStep(prePoint);
        TimeStep currTimeStep = this.createTimeStep(currPoint);
        if (currTimeStep == null || preTimeStep == null){
            return;
        }
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


        for (CandidatePoint cp1 : currTimeStep.getCandidates()){
            for (CandidatePoint cp2 : preTimeStep.getCandidates()){
                List<Double> temp = new ArrayList<>();
                //add weight
                if (cp1.equals(currSelectPoint) && cp2.equals(preSelectPoint)){
                    temp.add(1.0);
                }else {
                    temp.add(0.0);
                }
                //add avgDisBtwAvgDisBtw
                temp.add(sumBtwPreAndCurr);
                //add disBtwPreAndCurr
                temp.add(Math.abs(GeoFunctions.getDistanceInM(cp1.getLng(), cp1.getLat(), cp2.getLng(), cp2.getLat()) - sumBtwPreAndCurr));
                //add timeBtwPreAndCurr
                temp.add((currPoint.getTime().getTime() - prePoint.getTime().getTime()) / 1000.0);
                //add roadNum
                temp.add(currTimeStep.getCandidates().size() * 1.0);
                //add avgDistanceToRoad
                temp.add(sumToRoad);
                //add distanceToRoad
                temp.add(Math.abs(sumToRoad - GeoFunctions.getDistanceInM(cp1.getLng(), cp1.getLat(), currPoint.getLng(), currPoint.getLat())));
                //add bearing
                double obBearing = GeoFunctions.getBearing(currPoint.getLng(), currPoint.getLat(), prePoint.getLng(), prePoint.getLat());
                double candidateBearing = GeoFunctions.getBearing(cp1.getLng(), cp1.getLat(), cp2.getLng(), cp2.getLat());
                temp.add(Math.abs(obBearing - candidateBearing));
                allForAPoint.add(temp);
            }
        }
    }


}
