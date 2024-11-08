package group.algorithm.mapmatch.dwrmm;

import group.algorithm.mapmatch.tihmm.inner.SequenceState;
import group.algorithm.mapmatch.tihmm.inner.TimeStep;
import group.algorithm.shortestpath.AStarShortestPath;
import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;
import group.model.point.MapMatchedPoint;
import group.model.point.ProjectionPoint;
import group.model.roadnetwork.Path;
import group.model.roadnetwork.RoadNetwork;
import group.model.roadnetwork.RoadSegment;
import group.model.trajectory.MapMatchedTrajectory;
import group.model.trajectory.Trajectory;
import java.util.*;

import static group.model.point.CandidatePoint.calCandidatePoint;
import static group.util.GeoFunctions.getBearingDifference;
import static group.util.GeoFunctions.getDistanceInM;

public class DwrmmMapMatcher {

    protected final RoadNetwork roadNetwork;

    protected final AStarShortestPath aStarShortestPath;

    private TimeStep preTimeStep = null;

    private int count = 0;
    private double meanDistance = 0.0;
    private double sumDistance = 0.0;
    private double standardDeviation = 10.0;
    private double mean = 0.0;
    private double m2 = 0.0;
    private double meanDistanceDifference = 0.0;
    private double sumDistanceDifference = 0.0;

    public DwrmmMapMatcher(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.aStarShortestPath = new AStarShortestPath(roadNetwork);
    }

    private boolean isInitialized = false;
    private static final double K_INIT = 0.1;

    public MapMatchedTrajectory dwrmmMapMatch(Trajectory traj) {
        List<SequenceState> seq = new ArrayList<>();
        CandidatePoint skipPoint = new CandidatePoint();
        skipPoint.setSkip(true);
        seq.add(new SequenceState(skipPoint, traj.getGPSPointList().get(0)));

        for (int i = 1; i < traj.getGPSPointList().size(); i++) {
            GPSPoint p = traj.getGPSPointList().get(i);

            if (!isInitialized) {

                if (!delayedInitialization(traj.getGPSPointList().get(i - 1), p, seq, i)) {
                    continue;
                }
                isInitialized = true;
                continue;
            }

            mapMatch(p, seq, i);
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


    private boolean delayedInitialization(GPSPoint prevPoint, GPSPoint currentPoint, List<SequenceState> seq, int index) {

        TimeStep prevTimeStep = createTimeStep(prevPoint, index - 1);
        TimeStep currentTimeStep = createTimeStep(currentPoint, index);

        if (prevTimeStep == null || currentTimeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, currentPoint));
            return false;
        }

        double maxScore = Double.MIN_VALUE;
        CandidatePoint[] points = null;

        for (CandidatePoint prevCandidate : prevTimeStep.getCandidates()) {

            double maxConfidence = Double.MIN_VALUE;
            double[] bestScores = null;
            double[] secondBestScores = null;
            double bestTotalScore = Double.MIN_VALUE;
            double secondBestTotalScore = Double.MIN_VALUE;
            CandidatePoint preMatchedPoint = null;
            CandidatePoint matchedPoint = null;
            for (CandidatePoint currentCandidate : currentTimeStep.getCandidates()) {

                double distanceScore = calculateDistanceScore(getDistanceInM(prevPoint, currentPoint));

                double directionDifference = getBearingDifference(currentPoint, prevPoint, currentCandidate, prevCandidate);
                double directionWeight = calculateDirectionWeight(directionDifference);
                double directionScore = directionWeight * calculateDirectionScore(directionDifference);

                double linearDistance = getDistanceInM(currentPoint, prevPoint);
                Path path = aStarShortestPath.findShortestPath(currentCandidate, prevCandidate);
                double pathDistance = path.getLengthInMeter();
                double distanceDifference = Math.abs(linearDistance - pathDistance);
                double connectivityWeight = calculateConnectivityWeight(distanceDifference);
                double connectivityScore =  connectivityWeight * calculateConnectivityScore(distanceDifference);


                double totalScore = distanceScore + directionScore + connectivityScore;


                if (totalScore > bestTotalScore) {
                    preMatchedPoint = prevCandidate;
                    matchedPoint = currentCandidate;

                    secondBestTotalScore = bestTotalScore;
                    secondBestScores = bestScores;
                    bestTotalScore = totalScore;
                    bestScores = new double[]{distanceScore, directionScore, connectivityScore, 1.0, directionWeight, connectivityWeight};
                } else if (totalScore != bestTotalScore && totalScore > secondBestTotalScore) {

                    secondBestTotalScore = totalScore;
                    secondBestScores = new double[]{distanceScore, directionScore, connectivityScore};
                }
            }

            if (bestScores != null && secondBestScores != null) {
                double confidence = calculateConfidence(bestScores, secondBestScores);
                maxConfidence = Math.max(maxConfidence, confidence);
            }


            if (maxConfidence >= K_INIT) {
                if (bestTotalScore > maxScore) {
                    maxScore = bestTotalScore;
                    points = new CandidatePoint[]{preMatchedPoint, matchedPoint};
                }
            }
        }

        if (points != null) {
            seq.remove(seq.size() - 1);
            seq.add(new SequenceState(points[0], prevPoint));
            seq.add(new SequenceState(points[1], currentPoint));
            currentTimeStep.setMatch(points[1]);
            preTimeStep = currentTimeStep;
            return true;
        }
        else {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, currentPoint));
        }
        return false;
    }


    private double calculateConfidence(double[] bestScores, double[] secondBestScores) {
        double distDiff = Math.abs(bestScores[0] - secondBestScores[0]);
        double directionDiff = Math.abs(bestScores[1] - secondBestScores[1]);
        double connectivityDiff = Math.abs(bestScores[2] - secondBestScores[2]);

        double weightDistance = bestScores[3];
        double weightDirection = bestScores[4];
        double weightConnectivity = bestScores[5];

        return ((weightDistance * distDiff / (bestScores[0] + secondBestScores[0]))
                + (weightDirection * directionDiff / (bestScores[1] + secondBestScores[1]))
                + (weightConnectivity * connectivityDiff / (bestScores[2] + secondBestScores[2]))) / (weightDistance + weightDirection + weightConnectivity);
    }


    public void mapMatch(GPSPoint observation, List<SequenceState> seq, int index) {
        TimeStep timeStep = createTimeStep(observation, index);
        double matchedDistance = 0;
        double matchedDistanceDifference = 0;
        double[] bestScores = null;
        double[] secondBestScores = null;
        double bestTotalScore = Double.MIN_VALUE;
        double secondBestTotalScore = Double.MIN_VALUE;
        CandidatePoint matchedPoint = null;
        if (timeStep == null) {
            CandidatePoint skipPoint = new CandidatePoint();
            skipPoint.setSkip(true);
            seq.add(new SequenceState(skipPoint, observation));
            preTimeStep = null;
            return;
        }
        for (CandidatePoint candidatePoint : timeStep.getCandidates()) {
            double totalScore;
            double distanceScore;
            double directionScore = 0;
            double connectivityScore = 0;
            double directionWeight = 1.0;
            double connectivityWeight = 1.0;
            double distanceDifference = 0;


            RoadSegment roadSegment = roadNetwork.getRoadSegmentById(
                    candidatePoint.getRoadSegmentId()
            );
            ProjectionPoint projectionPoint = ProjectionPoint.calProjection(observation, roadSegment.getPoints(), 0, roadSegment.getPoints().size() - 1);
            distanceScore = calculateDistanceScore(projectionPoint.getErrorDistanceInMeter());

            if (preTimeStep == null) {
                totalScore = distanceScore;
            }
            else {

                GPSPoint preObservation = preTimeStep.getObservation();
                CandidatePoint preMatch = preTimeStep.getMatch();
                double directionDifference = getBearingDifference(observation, preObservation, candidatePoint, preMatch);
                if (directionDifference == 0) {
                    distanceScore = 0;
                }
                else {
                    directionWeight = calculateDirectionWeight(directionDifference);
                    directionScore = directionWeight * calculateDirectionScore(directionDifference);
                }


                double linearDistance = getDistanceInM(observation, preObservation);
                Path path = aStarShortestPath.findShortestPath(preMatch, candidatePoint);
                double pathDistance = path.getLengthInMeter();
                distanceDifference = Math.abs(linearDistance - pathDistance);
                connectivityWeight = calculateConnectivityWeight(distanceDifference);
                connectivityScore =  connectivityWeight * calculateConnectivityScore(distanceDifference);

                totalScore = distanceScore + directionScore + connectivityScore;
            }


            if (totalScore > bestTotalScore) {
                matchedPoint = candidatePoint;
                matchedDistance = projectionPoint.getErrorDistanceInMeter();
                matchedDistanceDifference = distanceDifference;

                secondBestTotalScore = bestTotalScore;
                secondBestScores = bestScores;
                bestTotalScore = totalScore;
                bestScores = new double[]{distanceScore, directionScore, connectivityScore, 1.0, directionWeight, connectivityWeight};
            } else if (totalScore != bestTotalScore && totalScore > secondBestTotalScore) {

                secondBestTotalScore = totalScore;
                secondBestScores = new double[]{distanceScore, directionScore, connectivityScore};
            }
        }


        if (bestScores != null && secondBestScores != null) {
            double confidence = calculateConfidence(bestScores, secondBestScores);
            if (confidence < K_INIT) {
                matchedPoint = calCandidatePoint(observation, roadNetwork.getRoadSegmentById(preTimeStep.getMatch().getRoadSegmentId()));
            }
        }


        count++;
        updateStandardDeviation(matchedDistance);
        if (preTimeStep != null) {
            updateMeanDistance(getDistanceInM(observation, preTimeStep.getObservation()));
            updateMeanDistanceDifference(matchedDistanceDifference);
        }

        seq.add(new SequenceState(matchedPoint, observation));
        timeStep.setMatch(matchedPoint);
        preTimeStep = timeStep;
    }


    public double calculateDistanceScore(double distance) {
        double sigmaG = getStandardDeviation();
        return (1 / (Math.sqrt(2 * Math.PI) * sigmaG)) * Math.exp(-(Math.pow(distance, 2) / (2 * Math.pow(sigmaG, 2))));
    }


    public double calculateDistanceWeight(double hdop, double HDOP1, double HDOP2) {
        if (hdop <= HDOP1) {
            return 1;
        } else if (hdop < HDOP2) {
            return (hdop - HDOP1) / (HDOP2 - HDOP1);
        } else {
            return 0;
        }
    }


    public double calculateHeadingScore(double thetaJi, double deltaTheta, int step) {
        double A = (step == 1) ? 1 : 0;
        double B = (step == 2) ? 1 : 0;
        return (1 + A * Math.cos(Math.toRadians(thetaJi)) + B * Math.cos(Math.toRadians(deltaTheta))) / 2;
    }


    public double calculateHeadingWeight(double velocity, double prevVelocity, double V1, double V2) {
        if (velocity < V1 || prevVelocity < V1) {
            return 0;
        } else if (velocity > V2 && prevVelocity > V2) {
            return 1;
        } else {
            return (velocity + prevVelocity - 2 * V1) / (2 * (V2 - V1));
        }
    }


    public double calculateDirectionScore(double betaJ) {
        double cosValue = Math.cos(Math.toRadians(betaJ));
        return (1 + cosValue) / 2;
    }


    public double calculateDirectionWeight(double distance) {
        if (count < 20) {
            return 1;
        }

        double meanDistance = getMeanDistance();
        if (distance > meanDistance) {
            return 1;
        } else {
            return distance / meanDistance;
        }
    }

    public double calculateConnectivityScore(double distanceDifference) {
        return Math.exp(-0.4 * distanceDifference);
    }

    public double calculateConnectivityWeight(double distanceDifference) {
        if (count < 20) {
            return 1.0;
        }

        double distanceDifferenceMean = getMeanDistanceDifference();
        if (distanceDifference <= distanceDifferenceMean) {
            return 1.0;
        } else {
            return calculateConnectivityScore(distanceDifference);
        }
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


    private void updateStandardDeviation(double newDistance) {
        if (count == 1) {
            return;
        }
        double delta = newDistance - mean;
        mean += delta / count;
        double delta2 = newDistance - mean;
        m2 += delta * delta2;
        standardDeviation = Math.sqrt(m2 / (count - 1));
    }


    private void updateMeanDistance(double newDistance) {
        if (count == 1) {
            return;
        }
        if (newDistance > 50) {
            newDistance = 50;
        }
        sumDistance += newDistance;
        meanDistance = sumDistance / (count - 1);
    }


    private void updateMeanDistanceDifference(double newDistanceDifference) {
        if (count == 1) {
            return;
        }
        if (newDistanceDifference > 50) {
            newDistanceDifference = 50;
        }
        sumDistanceDifference += newDistanceDifference;
        meanDistanceDifference = sumDistanceDifference / (count - 1);
    }


    private double getStandardDeviation() {
        return standardDeviation;
    }


    private double getMeanDistance() {
        return meanDistance;
    }


    private double getMeanDistanceDifference() {
        return meanDistanceDifference;
    }
}
