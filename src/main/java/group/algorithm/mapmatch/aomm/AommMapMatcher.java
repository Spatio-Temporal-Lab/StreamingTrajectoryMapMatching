package group.algorithm.mapmatch.aomm;

import group.algorithm.mapmatch.stream.StreamMapMatcher;
import group.algorithm.mapmatch.tihmm.inner.SequenceState;
import group.algorithm.mapmatch.tihmm.inner.TiViterbi;
import group.algorithm.mapmatch.tihmm.inner.TimeStep;
import group.algorithm.shortestpath.AStarShortestPath;
import group.algorithm.weightAdjuster.FixedWeightAdjuster;
import group.algorithm.weightAdjuster.WeightAdjuster;
import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;
import group.model.point.MapMatchedPoint;
import group.model.roadnetwork.Path;
import group.model.roadnetwork.RoadNetwork;
import group.model.trajectory.MapMatchedTrajectory;
import group.model.trajectory.Trajectory;
import group.util.GeoFunctions;

import scala.Tuple2;
import scala.Tuple3;


import java.util.*;


public class AommMapMatcher {

    protected final RoadNetwork roadNetwork;

    protected final AStarShortestPath aStarShortestPath;

    public AommMapMatcher(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.aStarShortestPath = new AStarShortestPath(roadNetwork);
    }

    private SequenceState originState;

    private TiViterbi originViterbi;

    private int recallNum;

    public MapMatchedTrajectory aommMapMatch(Trajectory traj) {

        LinkedList<TimeStep> preTimeSteps = new LinkedList<>();
        List<SequenceState> seq = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();
        FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();
        for (int i = 0; i < traj.getGPSPointList().size(); i++) {

            GPSPoint p = traj.getGPSPointList().get(i);
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> tuple2;
            recallNum = 0;
            tuple2 = this.computeViterbiSequence(p, seq, preTimeSteps, viterbi, i, fixedWeightAdjuster);
            seq = tuple2._1();
            TimeStep currentTimeStep = tuple2._2();
            viterbi = tuple2._3();

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

    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(
            GPSPoint point,
            List<SequenceState> seq,
            LinkedList<TimeStep> preTimeStepList,
            TiViterbi viterbi,
            int index,
            WeightAdjuster weightAdjuster
    ) {
        TimeStep timeStep = this.createTimeStep(point, index);
        TimeStep preTimeStep = null;
        if (!preTimeStepList.isEmpty()) {
            preTimeStep = preTimeStepList.getLast();
        }

        Map<Tuple2<CandidatePoint, CandidatePoint>, Path> shortestPathCache = new HashMap<>();

        // strategy 1
        if (timeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            seq.add(new SequenceState(skipPoint, point));
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
                    seq.add(new SequenceState(skipPoint, point));
                    return Tuple3.apply(seq, preTimeStep, viterbi);
                }
                // strategy 2
                else {

                    this.computeEmissionProbabilities(timeStep);

                    this.computeTransitionProbabilities(preTimeStep, timeStep, shortestPathCache);

                    viterbi.nextStep(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities(),
                            timeStep.getTransitionLogProbabilities(),
                            weightAdjuster
                    );

                    // strategy 3
                    CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);
                    timeStep.setMatch(maxPoint);
                    if (preTimeStep.getMatch() != null) {
                        Path path = shortestPathCache.getOrDefault(
                                new Tuple2<>(preTimeStep.getMatch(), maxPoint),
                                aStarShortestPath.findShortestPath(preTimeStep.getMatch(), maxPoint)
                        );
                        shortestPathCache.put(new Tuple2<>(preTimeStep.getMatch(), maxPoint), path);

                        double pathDistance = path.getLengthInMeter();
                        if (pathDistance > 100) {
                            if (recallNum == 0) {
                                originState = new SequenceState(maxPoint, point);
                                originViterbi = viterbi;
                            }
                            preTimeStepList.removeLast();
                            if (!preTimeStepList.isEmpty() && !viterbi.isBroken) {
                                recallNum++;
                                return computeViterbiSequence(point, seq, preTimeStepList, viterbi, index, weightAdjuster);
                            } else if (!originViterbi.isBroken) {
                                seq.add(originState);
                                return Tuple3.apply(seq, timeStep, originViterbi);
                            }
                        }
                    }
                    // strategy 3
                }
            } else {

                this.computeEmissionProbabilities(timeStep);
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
                CandidatePoint maxPoint = AommMapMatcher.findMaxValuePoint(viterbi.message);
                seq.add(new SequenceState(maxPoint, point));
                timeStep.setMatch(maxPoint);
            }
            preTimeStep = timeStep;
        }

        // strategy 3
        int j = 2;
        for (int i = 0; i < recallNum; ) {
            if (seq.get(seq.size() - j).getState().isSkip()) {
                j++;
            } else {
                seq.remove(seq.size() - j);
                i++;
            }
        }
        for (int i = 0; i < recallNum; i++) {
            CandidatePoint skipPoint = new CandidatePoint();
            seq.add(seq.size() - j, new SequenceState(skipPoint, point));
        }
        // strategy 3

        return Tuple3.apply(seq, preTimeStep, viterbi);
    }


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


    private TimeStep createTimeStep(GPSPoint pt, int index) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                50.0,
                index
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }

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


    protected void computeTransitionProbabilities(
            TimeStep prevTimeStep,
            TimeStep timeStep,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Path> shortestPathCache
    ) {

        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );

        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            double sum = 0;
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                Tuple2<CandidatePoint, CandidatePoint> key = new Tuple2<>(preCandiPt, curCandiPt);

                Path path = shortestPathCache.getOrDefault(key, aStarShortestPath.findShortestPath(preCandiPt, curCandiPt));
                shortestPathCache.putIfAbsent(key, path);

                double pathDistance = path.getLengthInMeter();
                sum += 1 / Math.abs(linearDist - pathDistance);
            }

            double mean = sum / timeStep.getCandidates().size();
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {
                Tuple2<CandidatePoint, CandidatePoint> key = new Tuple2<>(preCandiPt, curCandiPt);

                Path path = shortestPathCache.get(key);
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
