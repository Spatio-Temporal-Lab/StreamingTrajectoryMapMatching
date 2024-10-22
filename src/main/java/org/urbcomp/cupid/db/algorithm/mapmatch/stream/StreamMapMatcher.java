package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm.OnlineSequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm.OnlineViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.WeightAdjuster;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.GeoFunctions;
import scala.Tuple2;
import scala.Tuple3;

import java.util.*;

/**
 * StreamMapMatcher is responsible for performing map matching on trajectories
 * using various Viterbi algorithms.
 * <p>
 * This class leverages the road network and shortest path algorithms to align GPS points
 * with the road network based on probabilistic models.
 * </p>
 */
public class StreamMapMatcher {
    /**
     * Sigma parameter for the Gaussian distribution used in emission probabilities.
     */
    private static final double measurementErrorSigma = 50.0;

    /**
     * Beta parameter for the exponential distribution used in transition probabilities.
     */
    private static final double transitionProbabilityBeta = 5.0;

    /**
     * The road network on which the map matching is performed.
     */
    protected final RoadNetwork roadNetwork;

    /**
     * Shortest path algorithm for many-to-many pathfinding.
     */
    protected final AbstractManyToManyShortestPath pathAlgorithm;

    /**
     * Contains emission and transition probabilities for the Hidden Markov Model.
     */
    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );

    /**
     * Manages the bearing of the current window for processing.
     */
    private final WindowBearing windowBearing = new WindowBearing();

    /**
     * Bidirectional shortest path algorithm.
     */
    protected BidirectionalManyToManyShortestPath bidirectionalPathAlgorithm;

    /**
     * A list to store converged sequence states during processing.
     */
    public List<SequenceState> convergedSequence = new ArrayList<>();


    private Double delayTime = 0.0;

    private int delayNums = 0;
    /**
     * Constructs a StreamMapMatcher with the specified road network and path algorithm.
     *
     * @param roadNetwork   The road network used for map matching.
     * @param pathAlgorithm The shortest path algorithm for matching.
     */
    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgorithm) {
        this.roadNetwork = roadNetwork;
        this.pathAlgorithm = pathAlgorithm;
    }

    /**
     * Constructs a StreamMapMatcher with the specified road network, path algorithm, and bidirectional path algorithm.
     *
     * @param roadNetwork                The road network used for map matching.
     * @param pathAlgorithm              The shortest path algorithm for matching.
     * @param bidirectionalPathAlgorithm The bidirectional shortest path algorithm.
     */
    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgorithm, BidirectionalManyToManyShortestPath bidirectionalPathAlgorithm) {
        this.roadNetwork = roadNetwork;
        this.bidirectionalPathAlgorithm = bidirectionalPathAlgorithm;
        this.pathAlgorithm = pathAlgorithm;
    }

    /**
     * Finds the candidate point with the highest probability from the given map of candidate points.
     *
     * @param candidateMap A map containing candidate points and their associated probabilities.
     * @return The candidate point with the highest probability, or null if the map is empty or null.
     */
    public static CandidatePoint findMaxValuePoint(Map<CandidatePoint, Double> candidateMap) {
        CandidatePoint maxCandidate = null;
        double maxProbability = Double.MIN_VALUE;

        // Check if the candidate map is null
        if (candidateMap == null) {
            return maxCandidate;
        }

        // Iterate through the candidate map to find the candidate with the maximum probability
        for (Map.Entry<CandidatePoint, Double> entry : candidateMap.entrySet()) {
            CandidatePoint candidatePoint = entry.getKey();
            double probability = entry.getValue();

            // Update the max candidate if a new maximum is found
            if (maxCandidate == null || probability > maxProbability) {
                maxCandidate = candidatePoint;
                maxProbability = probability;
            }
        }
        return maxCandidate;
    }

    /**
     * Perform map matching on a trajectory using the stream map-matching method.
     *
     * @param trajectory Trajectory containing GPS points
     * @return MapMatchedTrajectory after matching
     * @throws AlgorithmExecuteException In case of algorithm errors
     */
    public MapMatchedTrajectory streamMapMatch(Trajectory trajectory, WeightAdjuster weightAdjuster) throws AlgorithmExecuteException {

        TimeStep previousTimeStep = null;
        List<SequenceState> sequence = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();

        int index = 0;

        for (GPSPoint gpsPoint : trajectory.getGPSPointList()) {
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> result =
                    this.computeViterbiSequence(gpsPoint, sequence, previousTimeStep, viterbi, index, weightAdjuster);
            index++;
            sequence = result._1();
            previousTimeStep = result._2();
            viterbi = result._3();
        }

        assert trajectory.getGPSPointList().size() == sequence.size();

        List<MapMatchedPoint> matchedPoints = new ArrayList<>(sequence.size());

        for (SequenceState state : sequence) {
            CandidatePoint candidate = state.getState() != null ? state.getState() : null;
            matchedPoints.add(new MapMatchedPoint(state.getObservation(), candidate));
        }

        return new MapMatchedTrajectory(trajectory.getTid(), trajectory.getOid(), matchedPoints);

    }

    /**
     * Perform online map matching on a trajectory using the online Viterbi method.
     *
     * @param trajectory Trajectory containing GPS points
     * @return MapMatchedTrajectory after online matching
     * @throws AlgorithmExecuteException In case of algorithm errors
     */
    public MapMatchedTrajectory onlineStreamMapMatch(Trajectory trajectory, WeightAdjuster weightAdjuster, int windowSize) throws AlgorithmExecuteException {

        TimeStep previousTimeStep = null;
        List<SequenceState> sequence = new ArrayList<>();
        OnlineViterbi viterbi = new OnlineViterbi(0, windowSize);

        int currentTime = 0;
        int trajectorySize = trajectory.getGPSPointList().size();
        int index = 0;

        for (GPSPoint gpsPoint : trajectory.getGPSPointList()) {
            Tuple3<List<SequenceState>, TimeStep, OnlineViterbi> result;

            // if state list is empty
            if (viterbi.getStateList().isEmpty()) {
                // currentTime = 0 indicates uninitialized
                // currentTime = 1 indicates initialized
                currentTime = (viterbi.message == null) ? 0 : 1;
            }

            result = this.computeOnlineViterbiSequence(gpsPoint, sequence, previousTimeStep, viterbi, weightAdjuster, currentTime, index);
            index++;
            sequence = result._1();
            previousTimeStep = result._2();
            viterbi = result._3();
            currentTime++;
        }

        assert trajectorySize == sequence.size();

        List<MapMatchedPoint> matchedPoints = new ArrayList<>(sequence.size());
        for (SequenceState state : sequence) {
            CandidatePoint candidate = state.getState() != null ? state.getState() : null;
            matchedPoints.add(new MapMatchedPoint(state.getObservation(), candidate));
        }

        return new MapMatchedTrajectory(trajectory.getTid(), trajectory.getOid(), matchedPoints);
    }

    /**
     * Compute a Viterbi sequence for a given GPS point.
     *
     * @param point        The GPS point to match
     * @param sequence     Current sequence of matched points
     * @param prevTimeStep Previous time step
     * @param viterbi      Viterbi object for sequence calculation
     * @return A tuple containing the updated sequence, time step, and Viterbi object
     * @throws AlgorithmExecuteException In case of errors
     */
    private Tuple3<List<SequenceState>, TimeStep, TiViterbi> computeViterbiSequence(
            GPSPoint point, List<SequenceState> sequence, TimeStep prevTimeStep, TiViterbi viterbi, int index, WeightAdjuster weightAdjuster)
            throws AlgorithmExecuteException {
        windowBearing.addPoint(point);
        TimeStep timeStep = this.createTimeStep(point, index);

        if (timeStep == null) {
            // No candidate points for this observation
            sequence.add(new SequenceState(null, point));
            // Reset Viterbi
            viterbi = new TiViterbi();
            prevTimeStep = null;
        } else {
            if (prevTimeStep != null) {
                Set<CandidatePoint> startPoints = new HashSet<>(prevTimeStep.getCandidates());
                Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());

                // Select corresponding shortest path algorithm
                Map<RoadNode, Map<RoadNode, Path>> paths = (bidirectionalPathAlgorithm == null)
                        ? pathAlgorithm.findShortestPath(startPoints, endPoints)
                        : bidirectionalPathAlgorithm.findShortestPath(startPoints, endPoints);

//                this.processBackward(prevTimeStep, timeStep, viterbi, paths);
                this.computeEmissionProbabilities(timeStep, probabilities);
                this.computeTransitionProbabilities(prevTimeStep, timeStep, probabilities, paths);
//                this.adjustWithDirection(timeStep, prevTimeStep, paths, probabilities);

                viterbi.nextStep(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities(),
                        timeStep.getTransitionLogProbabilities(),
                        weightAdjuster
                );
            } else {
                // Initialize probabilities for the first point
                this.computeEmissionProbabilities(timeStep, probabilities);//计算观测概率
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            }
            if (viterbi.isBroken) {
                // Handle broken Viterbi state
                sequence.add(viterbi.computeMostLikelySequence().get(viterbi.computeMostLikelySequence().size() - 1));
                viterbi = new TiViterbi();
                viterbi.startWithInitialObservation(
                        timeStep.getObservation(),
                        timeStep.getCandidates(),
                        timeStep.getEmissionLogProbabilities()
                );
            } else {
                CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);//找到最大概率的候选点
                sequence.add(new SequenceState(maxPoint, point));
            }
            prevTimeStep = timeStep;
        }
        return Tuple3.apply(sequence, prevTimeStep, viterbi);
    }

    /**
     * Computes a Viterbi sequence for a given GPS point.
     *
     * @param gpsPoint         The GPS point from the trajectory.
     * @param currentSequence  The current sequence of states.
     * @param previousTimeStep The previous time step for reference.
     * @param onlineViterbi    The Viterbi object used for calculations.
     * @param currentTime      The current time.
     * @return A tuple containing the updated sequence, previous time step, and Viterbi object.
     */
    private Tuple3<List<SequenceState>, TimeStep, OnlineViterbi> computeOnlineViterbiSequence(
            GPSPoint gpsPoint,
            List<SequenceState> currentSequence,
            TimeStep previousTimeStep,
            OnlineViterbi onlineViterbi,
            WeightAdjuster weightAdjuster,
            int currentTime,
            int index
    ) {
        System.out.println("Current time: " + currentTime);
        windowBearing.addPoint(gpsPoint);
        TimeStep currentTimeStep = this.createTimeStep(gpsPoint, index); // Create time step with GPS point and candidate set.

        int convergeStartIndex = onlineViterbi.getSequenceStates().size();

        if (currentTimeStep == null) {
            System.out.println("Current time step is null!");
            // Add the last element from the local sequence.
            currentSequence.add(new SequenceState(null, gpsPoint));

            // Record the start position for global sequence insertion.
            onlineViterbi = new OnlineViterbi(currentSequence.size(), onlineViterbi.windowSize);
            previousTimeStep = null;
        } else {
            if (previousTimeStep != null) {
                // Find the shortest path between candidate points of the previous and current time steps.
                Set<CandidatePoint> startPoints = new HashSet<>(previousTimeStep.getCandidates());
                Set<CandidatePoint> endPoints = new HashSet<>(currentTimeStep.getCandidates());

                Map<RoadNode, Map<RoadNode, Path>> paths = (bidirectionalPathAlgorithm == null)
                        ? pathAlgorithm.findShortestPath(startPoints, endPoints)
                        : bidirectionalPathAlgorithm.findShortestPath(startPoints, endPoints);

                this.processBackward(previousTimeStep, currentTimeStep, onlineViterbi, paths);
                // Calculate emission and transition probabilities
                this.computeEmissionProbabilities(currentTimeStep, probabilities);
                this.computeTransitionProbabilities(previousTimeStep, currentTimeStep, probabilities, paths);
                this.adjustWithDirection(currentTimeStep, previousTimeStep, paths, probabilities);

                onlineViterbi.nextStep(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities(),
                        currentTimeStep.getTransitionLogProbabilities(),
                        weightAdjuster,
                        currentTime
                );

            } else {
                // Initialize probabilities for the first GPS point.
                this.computeEmissionProbabilities(currentTimeStep, probabilities);
                onlineViterbi.startWithInitialObservation(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities()
                );
            }

            if (onlineViterbi.isBroken) {
                // Handle the case where the Viterbi algorithm encounters an issue.
                System.out.println("Viterbi is broken.");
                System.out.println("Broken sequence size: " + currentSequence.size());

                List<SequenceState> mostLikelySequence = onlineViterbi.computeMostLikelySequence();
                SequenceState lastState = mostLikelySequence.get(mostLikelySequence.size() - 1);
                currentSequence.add(lastState);

                // Record the start position for global sequence insertion.
                onlineViterbi = new OnlineViterbi(currentSequence.size(), onlineViterbi.windowSize, true);
                onlineViterbi.startWithInitialObservation(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities()
                );
            } else {
                if (onlineViterbi.isConverge) {
                    // Handle convergence of the Viterbi algorithm.
                    System.out.println("Viterbi has converged.");
                    System.out.println("======================================================");

                    List<OnlineSequenceState> localSequence = onlineViterbi.getSequenceStates();

                    int globalSeqInsertStartIndex = -1;
                    int localSeqInsertStartIndex = -1;
                    int earliestTime = currentTime - onlineViterbi.windowSize;
                    int correctedPoints = 0;
                    double traceDelay = 0;

                    convergeStartIndex = onlineViterbi.isConvergedBefore() ? convergeStartIndex : 0;

                    System.out.println("Local sequence size: " + localSequence.size());
                    System.out.println("Global sequence size: " + currentSequence.size());

                    // Determine insertion points for sequences
                    for (int i = convergeStartIndex; i < localSequence.size(); i++) {
                        GPSPoint localObservation = localSequence.get(i).getObservation();

                        for (int j = i + onlineViterbi.validStartInsertIndex; j < currentSequence.size(); j++) {
                            GPSPoint globalObservation = currentSequence.get(j).getObservation();

                            if (isSamePosition(localObservation, globalObservation)) {
                                localSeqInsertStartIndex = i;
                                globalSeqInsertStartIndex = j;
                                break;
                            }
                        }
                        if (localSeqInsertStartIndex != -1) break;
                    }

                    System.out.println("Converge start index: " + convergeStartIndex);
                    System.out.println("Global valid start index: " + onlineViterbi.validStartInsertIndex);
                    System.out.println("Global sequence insert start index: " + globalSeqInsertStartIndex);
                    System.out.println("Local sequence insert start index: " + localSeqInsertStartIndex);

                    if (localSeqInsertStartIndex != -1) {
                        int globalSeqInsertIndex = globalSeqInsertStartIndex;
                        int i = localSeqInsertStartIndex;
                        System.out.println("Expected Update size: " + (localSequence.size() - localSeqInsertStartIndex));

                        for (; i < localSequence.size(); i++) {
                            // 超过了当前序列的长度
                            if (globalSeqInsertIndex == currentSequence.size()) break;

                            OnlineSequenceState localState = localSequence.get(i);
                            SequenceState globalState = currentSequence.get(globalSeqInsertIndex);

                            GPSPoint localObservation = localState.getObservation();
                            GPSPoint globalObservation = globalState.getObservation();

                            CandidatePoint localCandidate = localState.getState();
                            CandidatePoint globalCandidate = globalState.getState();

                            // 回溯的候选点时间在窗口外
                            if (localState.time < earliestTime - onlineViterbi.windowSize) continue;
                            // 观测点的位置不同
                            if (!isSamePosition(localObservation, globalObservation)) continue;
                            // 经过回溯得到了正确匹配点
                            if (!isSamePosition(localCandidate, globalCandidate)) {
                                int delayTime = localState.time - earliestTime;
                                traceDelay += delayTime;
                                correctedPoints++;
                            }
                            currentSequence.set(globalSeqInsertIndex++, localState);
                        }

                        if (correctedPoints != 0 && traceDelay > 0) {
                            delayNums += correctedPoints;
                            delayTime += traceDelay;
                        }

                        System.out.println("Actual update size: " + (globalSeqInsertIndex - globalSeqInsertStartIndex));
                        System.out.println("The number of points that were correctly updated: " + correctedPoints);

                        // Record converged sequence.
                        convergedSequence.addAll(localSequence.subList(localSeqInsertStartIndex, i));

                    } else {
                        System.out.println("No corresponding sequence found.");
                    }

                    // Reset convergence state until the next convergence occurs.
                    onlineViterbi.isConverge = false;
                }

                // Find the candidate point with the maximum probability and add to the sequence.
                CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(onlineViterbi.message);
                currentSequence.add(new SequenceState(maxPoint, gpsPoint));
            }

            System.out.println("======================================================");

            previousTimeStep = currentTimeStep;
        }
        return Tuple3.apply(currentSequence, previousTimeStep, onlineViterbi);
    }

    /**
     * Handles the case where observation points may have shifted backward.
     *
     * @param preTimeStep The previous time step.
     * @param curTimeStep The current time step.
     * @param viterbi     The Viterbi object used for calculations.
     * @param paths       The shortest paths between candidate points.
     */
    public void processBackward(TimeStep preTimeStep, TimeStep curTimeStep,
                                TiViterbi viterbi, Map<RoadNode, Map<RoadNode, Path>> paths) {
        CandidatePoint preCandidatePoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);
        int roadSegmentId = preCandidatePoint.getRoadSegmentId();
        List<CandidatePoint> curCandidates = curTimeStep.getCandidates();
        boolean isMatch = false;

        for (CandidatePoint curCandiPt : curCandidates) {
            if (curCandiPt.getRoadSegmentId() == roadSegmentId &&
                    curCandiPt.getOffsetInMeter() < preCandidatePoint.getOffsetInMeter()) {

                RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(roadSegmentId);
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(curCandiPt.getRoadSegmentId());

                Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
                Path path = pathAlgorithm.getCompletePath(preCandidatePoint, curCandiPt, subPath);

                double speed = path.getLengthInMeter() * 1000 /
                        (curTimeStep.getObservation().getTime().getTime() -
                                preTimeStep.getObservation().getTime().getTime());

                if (speed > 34) {
                    double disBtwCurAndPer = GeoFunctions.getDistanceInM(preCandidatePoint, curCandiPt);
                    if (disBtwCurAndPer < 20) {
                        isMatch = true;
                        break;
                    }
                }
            }
        }

        if (isMatch) {
            curTimeStep.addCandidate(preCandidatePoint);
        }
    }

    /**
     * Adjusts transition probabilities based on directional information.
     *
     * @param currTimeStep  The current time step.
     * @param preTimeStep   The previous time step.
     * @param paths         The shortest paths between candidate points.
     * @param probabilities The probability distribution used for calculations.
     */
    public void adjustWithDirection(TimeStep currTimeStep, TimeStep preTimeStep,
                                    Map<RoadNode, Map<RoadNode, Path>> paths, HmmProbabilities probabilities) {
        if (!windowBearing.getChange()) {
            // Directional change is not detected; no adjustment needed.
            //System.out.println(windowBearing.getChange() + " " + windowBearing.getChangeScore() + " " + currTimeStep.getObservation());
        } else {
            GPSPoint currObservationPoint = currTimeStep.getObservation();
            GPSPoint preObservationPoint = preTimeStep.getObservation();

            double observationBearing = GeoFunctions.getBearing(
                    preObservationPoint.getLng(), preObservationPoint.getLat(),
                    currObservationPoint.getLng(), currObservationPoint.getLat()
            );
            double speed = GeoFunctions.getDistanceInM(
                    preObservationPoint.getLng(), preObservationPoint.getLat(),
                    currObservationPoint.getLng(), currObservationPoint.getLat()) * 1000 /
                    (currObservationPoint.getTime().getTime() - preObservationPoint.getTime().getTime());

            if (speed < 2.0) {
//                System.out.println(
//                        windowBearing.getChange() + " "
//                                + windowBearing.getChangeScore() + " "
//                                + "speed: " + speed + " "
//                                + currTimeStep.getObservation());
            } else {
                for (Map.Entry<Tuple2<CandidatePoint, CandidatePoint>, Double> entry :
                        currTimeStep.getTransitionLogProbabilities().entrySet()) {

                    Tuple2<CandidatePoint, CandidatePoint> key = entry.getKey();

                    RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(key._1.getRoadSegmentId());
                    RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(key._2.getRoadSegmentId());

                    Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
                    Path path = pathAlgorithm.getCompletePath(key._1, key._2, subPath);

                    double candidateBearing = path.calDisWeightDirection(roadNetwork);
                    double angleDifference = Math.abs(observationBearing - candidateBearing);

                    if (angleDifference > 180) {
                        angleDifference = 360 - angleDifference;
                    }

                    currTimeStep.getTransitionLogProbabilities().put(
                            key, currTimeStep.getTransitionLogProbabilities().get(key) +
                                    probabilities.directionLogProbability(angleDifference));
                }

//                System.out.println("true direction: " + windowBearing.getChangeScore() + " " + currTimeStep.getObservation());
            }
        }
    }

    /**
     * Calculates the emission probabilities based on the given time step and probability distribution.
     *
     * @param timeStep    The time step for which to calculate emission probabilities.
     * @param probability The established probability distribution used for calculations.
     */
    private void computeEmissionProbabilities(TimeStep timeStep, HmmProbabilities probability) {
        for (CandidatePoint candidate : timeStep.getCandidates()) {
            final double distance = candidate.getErrorDistanceInMeter();
            timeStep.addEmissionLogProbability(candidate, probability.emissionLogProbability(distance));
        }
    }

    /**
     * Computes the transition probabilities from the previous time step to the current time step.
     *
     * @param previousTimeStep The time step prior to the current one.
     * @param currentTimeStep  The current time step.
     * @param probabilities    The established probability distribution used for calculations.
     * @param paths            The shortest paths between candidate points.
     */
    protected void computeTransitionProbabilities(
            TimeStep previousTimeStep,
            TimeStep currentTimeStep,
            HmmProbabilities probabilities,
            Map<RoadNode, Map<RoadNode, Path>> paths
    ) {
        // Calculate the distance between observations.
        final double linearDistance = GeoFunctions.getDistanceInM(
                previousTimeStep.getObservation(),
                currentTimeStep.getObservation()
        );

        // Calculate transition probabilities for candidate points.
        for (CandidatePoint previousCandidate : previousTimeStep.getCandidates()) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(previousCandidate.getRoadSegmentId());

            for (CandidatePoint currentCandidate : currentTimeStep.getCandidates()) {
                RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(currentCandidate.getRoadSegmentId());
                Path subPath = paths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());
                Path completePath = pathAlgorithm.getCompletePath(previousCandidate, currentCandidate, subPath);
                double transitionLogProbability = probabilities.transitionLogProbability(completePath.getLengthInMeter(), linearDistance);

                if (completePath.getLengthInMeter() != Double.MAX_VALUE) {
                    currentTimeStep.addTransitionLogProbability(previousCandidate, currentCandidate, transitionLogProbability);
                }
            }
        }
    }

    /**
     * Creates a time step and initializes candidate points for the given GPS point.
     *
     * @param point The GPS point used to create the time step.
     * @return The created time step with initialized candidates, or null if no candidates are found.
     */
    private TimeStep createTimeStep(GPSPoint point, int index) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(point, roadNetwork, 50.0, index);

        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(point, candidates);
        }

        return timeStep;
    }

    /**
     * Checks if two GPS points are at the same position based on their longitude and latitude.
     *
     * @param p1 The first GPS point to compare.
     * @param p2 The second GPS point to compare.
     * @return True if both points are considered the same position, false otherwise.
     */
    private boolean isSamePosition(SpatialPoint p1, SpatialPoint p2) {
        return Math.abs(p1.getLng() - p2.getLng()) < 1e-6 && Math.abs(p1.getLat() - p2.getLat()) < 1e-6;
    }

    public Double getDelayTime() {
        return delayTime;
    }

    public int getDelayNums() {
        return delayNums;
    }
}
