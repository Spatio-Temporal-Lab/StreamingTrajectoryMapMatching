/*
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.algorithm.mapmatch.tihmm;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.DynamicWeightAdjuster;
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
import scala.Tuple2;

import java.util.*;

public class TiHmmMapMatcher {

    private static final double measurementErrorSigma = 50.0;

    private static final double transitionProbabilityBeta = 5.0;

    protected final RoadNetwork roadNetwork;

    protected final AbstractManyToManyShortestPath pathAlgo;
    private final WindowBearing windowBearing = new WindowBearing();

    public TiHmmMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public MapMatchedTrajectory mapMatch(Trajectory traj) {

        List<SequenceState> seq = this.computeViterbiSequence(traj.getGPSPointList());
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


    private TimeStep createTimeStep(GPSPoint pt, int index) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(
                pt,
                roadNetwork,
                measurementErrorSigma,
                index
        );
        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(pt, candidates);
        }
        return timeStep;
    }


    private List<SequenceState> computeViterbiSequence(List<GPSPoint> ptList) {
        List<SequenceState> seq = new ArrayList<>();
        final HmmProbabilities probabilities = new HmmProbabilities(
                measurementErrorSigma,
                transitionProbabilityBeta
        );
        TiViterbi viterbi = new TiViterbi();
        DynamicWeightAdjuster dynamicWeightAdjuster = new DynamicWeightAdjuster();
        TimeStep preTimeStep = null;
        int idx = 0;
        int nbPoints = ptList.size();
        while (idx < nbPoints) {
            windowBearing.addPoint(ptList.get(idx));
            TimeStep timeStep = this.createTimeStep(ptList.get(idx), idx);
            if (timeStep == null) {
                seq.addAll(viterbi.computeMostLikelySequence());
                seq.add(new SequenceState(null, ptList.get(idx)));
                viterbi = new TiViterbi();
                preTimeStep = null;
            } else {
                if (preTimeStep != null) {

                    Set<CandidatePoint> startPoints = new HashSet<>(preTimeStep.getCandidates());
                    Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());
                    Map<RoadNode, Map<RoadNode, Path>> paths = pathAlgo.findShortestPath(
                            startPoints,
                            endPoints
                    );

                    processBackward(preTimeStep, timeStep, viterbi, paths, probabilities);

                    this.computeEmissionProbabilities(timeStep, probabilities);

                    this.computeTransitionProbabilities(preTimeStep, timeStep, probabilities, paths);

                    this.adjustWithDirection(timeStep, preTimeStep, paths, probabilities);

                    viterbi.nextStep(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities(),
                            timeStep.getTransitionLogProbabilities(),
                            dynamicWeightAdjuster
                    );
                } else {

                    this.computeEmissionProbabilities(timeStep, probabilities);
                    viterbi.startWithInitialObservation(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities()
                    );
                }
                if (viterbi.isBroken) {
                    seq.addAll(viterbi.computeMostLikelySequence());
                    viterbi = new TiViterbi();
                    viterbi.startWithInitialObservation(
                            timeStep.getObservation(),
                            timeStep.getCandidates(),
                            timeStep.getEmissionLogProbabilities()
                    );
                }
                preTimeStep = timeStep;
            }
            idx += 1;
        }
        if (seq.size() < nbPoints) {
            seq.addAll(viterbi.computeMostLikelySequence());
        }
        return seq;
    }


    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep, TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths, HmmProbabilities probabilities) {
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
        GPSPoint currObPoint = currTimeStep.getObservation();
        GPSPoint preObPoint = preTimeStep.getObservation();
        double obBearing = GeoFunctions.getBearing(preObPoint.getLng(), preObPoint.getLat(), currObPoint.getLng(), currObPoint.getLat());
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

        final double linearDist = GeoFunctions.getDistanceInM(
                prevTimeStep.getObservation(),
                timeStep.getObservation()
        );


        for (CandidatePoint preCandiPt : prevTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    preCandiPt.getRoadSegmentId()
            );
            for (CandidatePoint curCandiPt : timeStep.getCandidates()) {


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

}
