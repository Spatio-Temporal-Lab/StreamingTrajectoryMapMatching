package org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm;

import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.ExtendedState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.ForwardStepResult;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.WeightAdjuster;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import scala.Tuple2;

import java.util.*;

import static org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher.findMaxValuePoint;

/**
 * Represents the Online Viterbi algorithm for processing
 * sequences of GPS observations to infer the most likely
 * states over time.
 */
public class OnlineViterbi extends TiViterbi {
    // Collection of Viterbi states
    private final LinkedList<OnlineExtendedState> stateList;
    // Local solutions for the current sequence
    private final List<OnlineSequenceState> sequenceStates;

    // Convergence status
    public boolean isConverge;
    // Whether the algorithm was broken before
    public boolean isBrokenBefore;

    // Current convergence point
    private OnlineExtendedState currentRoot;
    // Previous convergence point
    private OnlineExtendedState previousRoot;

    // Time diff between root and previous root
    public int timeDelta;
    // Starting insert position for global sequence after algorithm interruption
    public int validStartInsertIndex;
    // Boundary size of convergence from the current time step
    public int windowSize;

    /**
     * Constructs an OnlineViterbi instance with an initial insert position of zero.
     */
    public OnlineViterbi() {
        stateList = new LinkedList<>();
        sequenceStates = new ArrayList<>();
        isConverge = false;
        isBrokenBefore = false;
        currentRoot = null;
        previousRoot = null;
        timeDelta = 0;
        validStartInsertIndex = 0;
        windowSize = 0;
    }

    /**
     * Constructs an OnlineViterbi instance with a specified insert position.
     * For recording insert position of global sequence when algorithm breaks.
     *
     * @param validStartInsertIndex The starting insert position for the global sequence.
     */
    public OnlineViterbi(int validStartInsertIndex) {
        stateList = new LinkedList<>();
        sequenceStates = new ArrayList<>();
        isConverge = false;
        isBrokenBefore = false;
        currentRoot = null;
        previousRoot = null;
        timeDelta = 0;
        this.validStartInsertIndex = validStartInsertIndex;
        windowSize = 0;
    }

    /**
     * Constructs an OnlineViterbi instance with a specified insert position and broken state indicator.
     * This constructor is used for recording the insert position of the global sequence when the algorithm breaks.
     *
     * @param validStartInsertIndex The starting insert position for the global sequence.
     * @param isBrokenBefore        A boolean indicating whether the algorithm was broken before this instance was created.
     */
    public OnlineViterbi(int validStartInsertIndex, boolean isBrokenBefore) {
        stateList = new LinkedList<>();
        sequenceStates = new ArrayList<>();
        isConverge = false;
        this.isBrokenBefore = isBrokenBefore;
        currentRoot = null;
        previousRoot = null;
        timeDelta = 0;
        this.validStartInsertIndex = validStartInsertIndex;
        windowSize = 0;
    }

    public OnlineViterbi(int validStartInsertIndex, int windowSize) {
        stateList = new LinkedList<>();
        sequenceStates = new ArrayList<>();
        isConverge = false;
        this.isBrokenBefore = false;
        currentRoot = null;
        previousRoot = null;
        timeDelta = 0;
        this.validStartInsertIndex = validStartInsertIndex;
        this.windowSize = windowSize;
    }

    public OnlineViterbi(int validStartInsertIndex, int windowSize, boolean isBrokenBefore) {
        stateList = new LinkedList<>();
        sequenceStates = new ArrayList<>();
        isConverge = false;
        this.isBrokenBefore = isBrokenBefore;
        currentRoot = null;
        previousRoot = null;
        timeDelta = 0;
        this.validStartInsertIndex = validStartInsertIndex;
        this.windowSize = windowSize;
    }


    /**
     * Processes the next step in the Viterbi algorithm using the given observation
     * and candidate points, updating the internal state accordingly.
     *
     * @param observation                The current GPS observation.
     * @param candidates                 List of candidate points for the current step.
     * @param emissionLogProbabilities   Map of emission log probabilities for candidate points.
     * @param transitionLogProbabilities Map of transition log probabilities between candidate points.
     * @param time                       The current time step.
     * @throws IllegalStateException if the method is called without initializing
     *                               with an observation or after an HMM break.
     */
    public void nextStep(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            WeightAdjuster weightAdjuster,
            int time
    ) {
        if (message == null) throw new IllegalStateException("start with initial observation() must be called first.");
        if (isBroken) throw new IllegalStateException("Method must not be called after an HMM break.");

        // Call forward computation
        ForwardStepResult forwardStepResult = forwardStep(
                observation,
                prevCandidates,
                candidates,
                message,
                emissionLogProbabilities,
                transitionLogProbabilities,
                weightAdjuster,
                time
        );

        isBroken = hmmBreak(forwardStepResult.getNewMessage());
        if (isBroken) {
            return;
        }

        // Update message
        message = forwardStepResult.getNewMessage();
        lastExtendedStates = forwardStepResult.getNewExtendedStates();
        prevCandidates = new ArrayList<>(candidates);
    }

    /**
     * Starts Viterbi calculation and extends forward based on the given observation.
     *
     * @param observation                The original trajectory point.
     * @param prevCandidates             List of previous candidate points.
     * @param curCandidates              List of current candidate points.
     * @param message                    Map of state probabilities.
     * @param emissionLogProbabilities   Map of emission probabilities for each candidate.
     * @param transitionLogProbabilities Map of transition probabilities between candidates.
     * @param time                       The current time step.
     * @return Results after forward extension, including updated state probabilities and states.
     */
    protected ForwardStepResult forwardStep(
            GPSPoint observation,
            List<CandidatePoint> prevCandidates,
            List<CandidatePoint> curCandidates,
            Map<CandidatePoint, Double> message,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            WeightAdjuster weightAdjuster,
            int time
    ) {
        assert !prevCandidates.isEmpty();

        // Number of current candidates
        int currCandiSize = curCandidates.size();
        // Result of forward computation
        final ForwardStepResult result = new ForwardStepResult(currCandiSize);

        // Accumulators for current time step probabilities
        double accumulatedEmissionLogProb = 0.0;
        double accumulatedTransitionLogProb = 0.0;

        // Valid state count at current time step
        int validStateCount = 0;
        // Last state in the sequence
        OnlineExtendedState lastState = null;
        ListIterator<OnlineExtendedState> iterator;

        if (!stateList.isEmpty()) lastState = stateList.getLast();

        for (CandidatePoint curState : curCandidates) {
            double maxTransitionLogProb = Double.NEGATIVE_INFINITY;
            double maxLogProb = Double.NEGATIVE_INFINITY;
            CandidatePoint maxPreState = null;

            for (CandidatePoint preState : prevCandidates) {
                double transitionLogProb = transitionLogProbability(
                        preState,
                        curState,
                        transitionLogProbabilities
                );
                final double logProb = message.get(preState) + weightAdjuster.getTransitionWeight() * transitionLogProb;
                if (logProb > maxLogProb) {
                    maxTransitionLogProb = transitionLogProb;
                    maxLogProb = logProb;
                    maxPreState = preState;
                }
            }

            double emissionLogProb = emissionLogProbabilities.get(curState);
            result.getNewMessage().put(curState, (maxLogProb + weightAdjuster.getEmissionWeight() * emissionLogProb));

            if (maxTransitionLogProb != Double.NEGATIVE_INFINITY && emissionLogProb != Double.NEGATIVE_INFINITY) {
                accumulatedEmissionLogProb += emissionLogProb;
                accumulatedTransitionLogProb += maxTransitionLogProb;
            }

            // If there is a maximum probability state from the previous time step
            if (maxPreState != null) {
                OnlineExtendedState onlineExtendedState = null;

                /*
                    Determine the state based on time
                        =0: initialization
                        =1: first time step after initialize
                        >1: time steps after first step
                */
                if (time == 1)
                    onlineExtendedState = new OnlineExtendedState(
                            curState, lastExtendedStates.get(maxPreState), observation,
                            time, 0, currCandiSize, null);

                else if (time > 1) {
                    OnlineExtendedState parentState = lastState;
                    iterator = stateList.listIterator(stateList.indexOf(lastState));

                    assert parentState != null;

                    // Find the parent node from the previous time step
                    while (parentState.getTime() != time - 1 || parentState.getState() != maxPreState)
                        parentState = iterator.previous();

                    // Increment the child count
                    parentState.setNumOfChild(parentState.getNumOfChild() + 1);
                    onlineExtendedState = new OnlineExtendedState(
                            curState, lastExtendedStates.get(maxPreState), observation,
                            time, 0, currCandiSize, parentState);
                }

                // Add the extended state to the results
                result.getNewExtendedStates().put(curState, onlineExtendedState);
                // Add the new state to the state list
                stateList.add(onlineExtendedState);
                // Increment valid state count
                validStateCount++;
            }
        }

        // Update weights after processing all current states
        if (accumulatedTransitionLogProb != Double.NEGATIVE_INFINITY &&
                accumulatedEmissionLogProb != Double.NEGATIVE_INFINITY) {
            weightAdjuster.updateWeights(accumulatedEmissionLogProb, accumulatedTransitionLogProb);
        }

        // If there are valid states and the state list is not empty
        if (!stateList.isEmpty() && validStateCount > 0) {
            lastState = stateList.getLast();
            iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);

            // Update valid state count for the current time step
//            System.out.println("valid count: " + validStateCount);
            for (int i = 0; i < validStateCount; i++) {
                OnlineExtendedState current = iterator.previous();
                current.setNumOfState(validStateCount);
            }

            // Compress unnecessary states
            compress(time);
            // Free up dummy states
            freeDummyState(time);
            // Check for convergence point and record local solutions
            if (searchForNewRoot()) {
                isConverge = true;
                traceback();
            }
        }

        return result;
    }

    /**
     * Compresses the state tree to remove unnecessary nodes
     * that do not contribute to the current path, thereby
     * optimizing the state representation.
     *
     * @param currTime The current time to consider for pruning.
     */
    private void compress(int currTime) {
        OnlineExtendedState lastState = stateList.getLast();
        ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);

        while (iterator.hasPrevious()) {
            OnlineExtendedState current = iterator.previous();
            int time = current.getTime();
            int numOfChild = current.getNumOfChild();
            OnlineExtendedState parent = current.getParent();

            if (numOfChild == 0 && time != currTime) {
                // Remove child nodes if there are no children and it's not the current time
                if (parent != null) parent.setNumOfChild(parent.getNumOfChild() - 1);
            } else {
                // Shrink the parent node if it has only one child
                while (parent != null && parent.getNumOfChild() == 1) {
                    current.setParent(current.getParent().getParent());
                    parent = current.getParent();
                }
            }
        }
    }

    /**
     * Frees up dummy states that are no longer needed in the
     * state tree, specifically those that do not have any
     * children and are not at the current time.
     *
     * @param currTime The current time to consider for removal.
     */
    private void freeDummyState(int currTime) {
        OnlineExtendedState lastState = stateList.getLast();
        ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);
        OnlineExtendedState current;

        while (iterator.hasPrevious()) {
            current = iterator.previous();
            int time = current.getTime();
            int numOfChild = current.getNumOfChild();
            // Remove states with no children and that are not at the current time
            if (numOfChild <= 0 && time != currTime) iterator.remove();
        }
    }

    /**
     * Searches for a new root (convergence point) in the state tree.
     * If a convergence point is found, it updates the current root.
     *
     * @return true if a new root was found, false otherwise.
     */
    private boolean searchForNewRoot() {
        // Search for a convergence point
        if (currentRoot == null) {
            OnlineExtendedState lastState = stateList.getLast();
            ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);
            OnlineExtendedState ancestor = null;
            int currCandiSize = lastState.getNumOfState();

            // Backtrack to the same ancestor node
            for (int i = 0; i < currCandiSize; i++) {
                OnlineExtendedState current = iterator.previous();
                while (current != null) {
                    OnlineExtendedState prev = current;
                    current = current.getParent();
                    if (current == null) {
                        if (ancestor == null) ancestor = prev;
                        else {
                            if (ancestor != prev) return false;
                        }
                    }
                }
            }
        }

        // There is a convergence point
        OnlineExtendedState current = stateList.getLast();
        OnlineExtendedState ancestor = null;

        // Time difference between the current and previous convergence points
        timeDelta = current.getTime();

        while (current != null) {
            if (current.getNumOfChild() >= 2) ancestor = current;
            current = current.getParent();
        }

        if (ancestor != null) {
            if (currentRoot == null) {
                currentRoot = ancestor;
                timeDelta = timeDelta - ancestor.getTime();
                return timeDelta != 0;
            } else {
                if (ancestor != currentRoot) {
                    timeDelta = timeDelta - ancestor.getTime();
                    if (timeDelta == 0) return false;
                    else {
                        previousRoot = currentRoot;
                        currentRoot = ancestor;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /**
     * Performs traceback to find the local sequence from the
     * current root state to the previous root state.
     */
    private void traceback() {
        int currentTime = currentRoot.getTime();

        System.out.println("Local path found!");
        System.out.println("Current root time: " + currentTime);

        List<OnlineSequenceState> localSequence = new ArrayList<>();
        localSequence.add(new OnlineSequenceState(currentRoot.getState(), currentRoot.getObservation(), currentTime--));

        int depth = isConvergedBefore() ? currentRoot.getTime() - previousRoot.getTime() - 1 : currentRoot.getTime();
        ExtendedState current = currentRoot.getBackPointer();

        for (int i = 0; i < depth; i++) {
            localSequence.add(new OnlineSequenceState(current.getState(), current.getObservation(), currentTime--));
            current = current.getBackPointer();
        }

        assert current == null || current.getState() == previousRoot.getState();

        Collections.reverse(localSequence);
        sequenceStates.addAll(localSequence);

        System.out.println("Local added sequence length: " + localSequence.size());
    }

    /**
     * Performs traceback for the last part of the path,
     * finding the sequence states from the last extended state
     * to the current root.
     *
     * @param observation The GPS point observation to include in the path.
     */
    public void tracebackLastPart(GPSPoint observation) {
        System.out.println("traceback last part");

        List<OnlineSequenceState> interLocalPath = new ArrayList<>();
        CandidatePoint maxValuePoint = findMaxValuePoint(message);
        interLocalPath.add(new OnlineSequenceState(maxValuePoint, observation));

        if (lastExtendedStates == null) return;
        ExtendedState current = lastExtendedStates.get(maxValuePoint);

        while (current != currentRoot) {
            interLocalPath.add(new OnlineSequenceState(current.getState(), current.getObservation()));
            current = current.getBackPointer();
        }

        Collections.reverse(interLocalPath);
        sequenceStates.addAll(interLocalPath);
    }

    /**
     * Returns the list of online extended states.
     *
     * @return A LinkedList of OnlineExtendedState objects.
     */
    public LinkedList<OnlineExtendedState> getStateList() {
        return stateList;
    }

    /**
     * Returns the list of sequence states.
     *
     * @return A List of SequenceState objects.
     */
    public List<OnlineSequenceState> getSequenceStates() {
        return sequenceStates;
    }

    /**
     * Checks if a convergence has occurred before.
     *
     * @return true if a previous root exists, false otherwise.
     */
    public boolean isConvergedBefore() {
        return previousRoot != null;
    }
}
