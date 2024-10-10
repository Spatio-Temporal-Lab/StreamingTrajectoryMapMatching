package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

import org.urbcomp.cupid.db.algorithm.bearing.WindowBearing;
import org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm.OnlineViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TimeStep;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
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
import scala.Tuple2;
import scala.Tuple3;

import java.util.*;

public class StreamMapMatcher {
    /**
     * Sigma parameter for the Gaussian distribution used in emission probabilities
     */
    private static final double measurementErrorSigma = 50.0;

    /**
     * Beta parameter for the exponential distribution used in transition probabilities
     */
    private static final double transitionProbabilityBeta = 5.0;

    /**
     * Road network
     */
    protected final RoadNetwork roadNetwork;
    protected final AbstractManyToManyShortestPath pathAlgorithm;
    final HmmProbabilities probabilities = new HmmProbabilities(
            measurementErrorSigma,
            transitionProbabilityBeta
    );
    private final WindowBearing windowBearing = new WindowBearing();
    protected BidirectionalManyToManyShortestPath bidirectionalPathAlgorithm;

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgorithm) {
        this.roadNetwork = roadNetwork;
        this.pathAlgorithm = pathAlgorithm;
    }

    public StreamMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgorithm, BidirectionalManyToManyShortestPath bidirectionalPathAlgorithm) {
        this.roadNetwork = roadNetwork;
        this.bidirectionalPathAlgorithm = bidirectionalPathAlgorithm;
        this.pathAlgorithm = pathAlgorithm;
    }

    /**
     * Find the candidate point with the highest probability from the map.
     *
     * @param candidateMap Map containing candidate points and their probabilities
     * @return The candidate point with the highest probability
     */
    public static CandidatePoint findMaxValuePoint(Map<CandidatePoint, Double> candidateMap) {
        CandidatePoint maxCandidate = null;
        double maxProbability = Double.MIN_VALUE;

        if (candidateMap == null) {
            return maxCandidate;
        }

        for (Map.Entry<CandidatePoint, Double> entry : candidateMap.entrySet()) {
            CandidatePoint candidatePoint = entry.getKey();
            double probability = entry.getValue();

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
    public MapMatchedTrajectory streamMapMatch(Trajectory trajectory) throws AlgorithmExecuteException {

        TimeStep previousTimeStep = null;
        List<SequenceState> sequence = new ArrayList<>();
        TiViterbi viterbi = new TiViterbi();


        for (GPSPoint gpsPoint : trajectory.getGPSPointList()) {
            Tuple3<List<SequenceState>, TimeStep, TiViterbi> result =
                    this.computeViterbiSequence(gpsPoint, sequence, previousTimeStep, viterbi);

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
    public MapMatchedTrajectory onlineStreamMapMatch(Trajectory trajectory) throws AlgorithmExecuteException {

        TimeStep previousTimeStep = null;
        List<SequenceState> sequence = new ArrayList<>();
        OnlineViterbi viterbi = new OnlineViterbi();

        int currentTime = 0;
        int trajectorySize = trajectory.getGPSPointList().size();

        for (GPSPoint gpsPoint : trajectory.getGPSPointList()) {
            Tuple3<List<SequenceState>, TimeStep, OnlineViterbi> result;

            // if state list is empty
            if (viterbi.getStateList().isEmpty()) {
                // currentTime = 0 indicates uninitialized
                // currentTime = 1 indicates initialized
                currentTime = (viterbi.message == null) ? 0 : 1;
            }

            result = this.computeViterbiSequence(gpsPoint, sequence, previousTimeStep, viterbi, currentTime);

            sequence = result._1();
            previousTimeStep = result._2();
            viterbi = result._3();
            currentTime++;
        }

        System.out.println("trajectory size: " + trajectorySize);
        System.out.println("matched sequence size: " + sequence.size());
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
            GPSPoint point, List<SequenceState> sequence, TimeStep prevTimeStep, TiViterbi viterbi)
            throws AlgorithmExecuteException {
        windowBearing.addPoint(point);
        TimeStep timeStep = this.createTimeStep(point);

        if (timeStep == null) {
            sequence.add(new SequenceState(null, point)); // No candidate points for this observation
            viterbi = new TiViterbi(); // Reset Viterbi
            prevTimeStep = null;
        } else {
            if (prevTimeStep != null) {
                Set<CandidatePoint> startPoints = new HashSet<>(prevTimeStep.getCandidates());
                Set<CandidatePoint> endPoints = new HashSet<>(timeStep.getCandidates());

                // Select corresponding shortest path algorithm
                Map<RoadNode, Map<RoadNode, Path>> paths = (bidirectionalPathAlgorithm == null)
                        ? pathAlgorithm.findShortestPath(startPoints, endPoints)
                        : bidirectionalPathAlgorithm.findShortestPath(startPoints, endPoints);

//                this.processBackward(preTimeStep, timeStep, viterbi, paths);
                this.computeEmissionProbabilities(timeStep, probabilities);
                this.computeTransitionProbabilities(prevTimeStep, timeStep, probabilities, paths);
//                this.adjustWithDirection(timeStep, preTimeStep, paths, probabilities);

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
     * @param point       The GPS point from the trajectory.
     * @param seq         The current sequence of states.
     * @param preTimeStep The previous time step for reference.
     * @param viterbi     The Viterbi object used for calculations.
     * @param time        The current time index.
     * @return A tuple containing the updated sequence, previous time step, and Viterbi object.
     */
    private Tuple3<List<SequenceState>, TimeStep, OnlineViterbi> computeViterbiSequence(
            GPSPoint point,
            List<SequenceState> seq,
            TimeStep preTimeStep,
            OnlineViterbi viterbi,
            int time
    ) {
        System.out.println("current time: " + time);
        windowBearing.addPoint(point);
        TimeStep currentTimeStep = this.createTimeStep(point); // Create time step with point and candidate set.

        int convergeStartIndex = viterbi.getSequenceStates().size();

        if (currentTimeStep == null) {
            System.out.println("curr time step is null!");

            System.out.println("======================================================");
            System.out.println("Sequence length before traceback last part: " + seq.size());
            updateSequenceAfterTraceback(viterbi, point, seq);

            // Add the last element from the local sequence.
            seq.add(new SequenceState(null, point));
            System.out.println("Sequence length after traceback last part: " + seq.size());
            System.out.println("======================================================");

            // Record the start position for global sequence insertion.
            viterbi = new OnlineViterbi(seq.size());
            preTimeStep = null;
        } else {
            if (preTimeStep != null) {
                // Find the shortest path between candidate points of the previous and current time steps.
                Set<CandidatePoint> startPoints = new HashSet<>(preTimeStep.getCandidates());
                Set<CandidatePoint> endPoints = new HashSet<>(currentTimeStep.getCandidates());

                Map<RoadNode, Map<RoadNode, Path>> paths = (bidirectionalPathAlgorithm == null)
                        ? pathAlgorithm.findShortestPath(startPoints, endPoints)
                        : bidirectionalPathAlgorithm.findShortestPath(startPoints, endPoints);

//                this.processBackward(preTimeStep, currentTimeStep, viterbi, paths);
                this.computeEmissionProbabilities(currentTimeStep, probabilities);
                this.computeTransitionProbabilities(preTimeStep, currentTimeStep, probabilities, paths);
//                this.adjustWithDirection(currentTimeStep, preTimeStep, paths, probabilities);

                viterbi.nextStep(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities(),
                        currentTimeStep.getTransitionLogProbabilities(),
                        time
                );

            } else {
                // Initialize probabilities for the first point.
                this.computeEmissionProbabilities(currentTimeStep, probabilities);
                viterbi.startWithInitialObservation(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities()
                );
            }

            if (viterbi.isBroken) {
                // Handle the case where the Viterbi algorithm encounters an issue.
                System.out.println("Viterbi is broken.");
                System.out.println("======================================================");
                System.out.println("Sequence length before traceback last part: " + seq.size());

                updateSequenceAfterTraceback(viterbi, point, seq);

                List<SequenceState> localSequenceStates = viterbi.getSequenceStates();
                System.out.println("Local sequence length: " + localSequenceStates.size());

                // Add the second last element from the local sequence.
                seq.add(localSequenceStates.get(localSequenceStates.size() - 2));
                System.out.println("Sequence length after traceback last part: " + seq.size());
                System.out.println("======================================================");

                // Record the start position for global sequence insertion.
                viterbi = new OnlineViterbi(seq.size(), true);
                viterbi.startWithInitialObservation(
                        currentTimeStep.getObservation(),
                        currentTimeStep.getCandidates(),
                        currentTimeStep.getEmissionLogProbabilities()
                );
            } else {
                if (viterbi.isConverge) {
                    // Handle convergence of the Viterbi algorithm.
                    System.out.println("Viterbi has converged.");
                    System.out.println("======================================================");
                    System.out.println("Sequence length before merging converge part: " + seq.size());

                    List<SequenceState> sequenceStates = viterbi.getSequenceStates();
                    int size = sequenceStates.size();
                    System.out.println("Local sequence length: " + size);
                    System.out.println("Insert position: " + viterbi.insertPosition);
                    System.out.println("Converge start index: " + convergeStartIndex);

                    // 之前算法没有发生过中断，第一次收敛的序列从[index==0]开始复制（初始化的元素需要添加）
                    // 之前算法如果发生过中断，第一次收敛的序列从[index==1]开始复制（初始化的元素不需要添加）
                    // 非第一次收敛，从上一个时间的[preSeq.size()]开始复制，直到[currSeq.size()]
                    int isBrokenBefore = viterbi.isBrokenBefore ? 1 : 0;
                    convergeStartIndex = viterbi.isConvergedBefore() ? convergeStartIndex : isBrokenBefore;

                    for (int i = convergeStartIndex; i < size; i++) {
                        // 算法中断前，从索引0开始复制，无需减1
                        // 算法中断后，从索引1开始复制，需要减1
                        int insertPosition = viterbi.isBrokenBefore ? i + viterbi.insertPosition - 1 : i + viterbi.insertPosition;
                        if (i == convergeStartIndex) System.out.println("insert position: " + insertPosition);
                        seq.set(insertPosition, sequenceStates.get(i));
                    }

                    // Reset convergence state until the next convergence occurs.
                    viterbi.isConverge = false;

                    System.out.println("Sequence length after merging converge part: " + seq.size());
                    System.out.println("################################");
                }

                // Find the candidate point with the maximum probability and add to the sequence.
                CandidatePoint maxPoint = StreamMapMatcher.findMaxValuePoint(viterbi.message);
                seq.add(new SequenceState(maxPoint, point));
            }

            System.out.println("Sequence length: " + seq.size());
            preTimeStep = currentTimeStep;
        }
        return Tuple3.apply(seq, preTimeStep, viterbi);
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
            System.out.println(windowBearing.getChange() + " " + windowBearing.getChangeScore() + " " + currTimeStep.getObservation());
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
                System.out.println(
                        windowBearing.getChange() + " "
                                + windowBearing.getChangeScore() + " "
                                + "speed: " + speed + " "
                                + currTimeStep.getObservation());
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
    private TimeStep createTimeStep(GPSPoint point) {
        TimeStep timeStep = null;
        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(point, roadNetwork, 50.0);

        if (!candidates.isEmpty()) {
            timeStep = new TimeStep(point, candidates);
        }

        return timeStep;
    }

    /**
     * Updates the given sequence with the local sequence states after performing a traceback.
     *
     * @param viterbi The Viterbi instance used for traceback.
     * @param point   The point to be traced back.
     * @param seq     The sequence to be updated.
     */
    private void updateSequenceAfterTraceback(OnlineViterbi viterbi, GPSPoint point, List<SequenceState> seq) {
        viterbi.tracebackLastPart(point);
        List<SequenceState> localSequenceStates = viterbi.getSequenceStates();
        System.out.println("Local sequence length: " + localSequenceStates.size());
        int startIndex = seq.size() - localSequenceStates.size() + 1;

        // Update elements in the sequence.
        for (int i = startIndex; i < seq.size(); i++) {
            seq.set(i, localSequenceStates.get(i - startIndex));
        }
    }
}
