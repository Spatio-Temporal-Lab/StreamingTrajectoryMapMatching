package group.algorithm.mapmatch.onlinemm;

import group.algorithm.mapmatch.tihmm.inner.ExtendedState;
import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;

/**
 * Represents an extended Viterbi state that includes additional
 * information for online Viterbi algorithm, such as time,
 * number of child states, and parent state.
 */
public class OnlineExtendedState extends ExtendedState {

    // Time associated with this state
    private int time;
    // Number of child states for this state
    private int numOfChild;
    // Number of valid states at the current time step
    private int numOfState;
    // Parent state in the Viterbi sequence
    private OnlineExtendedState parent;

    /**
     * Constructs an OnlineExtendedState with the specified candidate point,
     * backpointer, and observation.
     *
     * @param state       The candidate point for this state.
     * @param backPointer The backpointer to the previous state.
     * @param observation The original GPS point (observation).
     */
    public OnlineExtendedState(CandidatePoint state, ExtendedState backPointer, GPSPoint observation) {
        super(state, backPointer, observation);
    }

    /**
     * Constructs an OnlineExtendedState with additional parameters, including time,
     * number of child states, number of valid states, and parent state.
     *
     * @param state       The candidate point for this state.
     * @param backPointer The back pointer to the previous state.
     * @param observation The original GPS point (observation).
     * @param time        The time associated with this state.
     * @param numOfChild  The number of child states for this state.
     * @param numOfState  The number of valid states at the current time step.
     * @param parent      The parent state in the Viterbi sequence.
     */
    public OnlineExtendedState(CandidatePoint state, ExtendedState backPointer, GPSPoint observation,
                               int time, int numOfChild, int numOfState, OnlineExtendedState parent) {
        super(state, backPointer, observation);
        this.time = time;
        this.numOfChild = numOfChild;
        this.numOfState = numOfState;
        this.parent = parent;
    }

    /**
     * Gets the time associated with this state.
     *
     * @return The time for this state.
     */
    public int getTime() {
        return time;
    }

    /**
     * Sets the time for this state.
     *
     * @param time The new time to set.
     */
    public void setTime(int time) {
        this.time = time;
    }

    /**
     * Gets the number of child states for this state.
     *
     * @return The number of child states.
     */
    public int getNumOfChild() {
        return numOfChild;
    }

    /**
     * Sets the number of child states for this state.
     *
     * @param numOfChild The new number of child states to set.
     */
    public void setNumOfChild(int numOfChild) {
        this.numOfChild = numOfChild;
    }

    /**
     * Gets the number of valid states at the current time step.
     *
     * @return The number of valid states.
     */
    public int getNumOfState() {
        return numOfState;
    }

    /**
     * Sets the number of valid states at the current time step.
     *
     * @param numOfState The new number of valid states to set.
     */
    public void setNumOfState(int numOfState) {
        this.numOfState = numOfState;
    }

    /**
     * Gets the parent state in the Viterbi sequence.
     *
     * @return The parent state.
     */
    public OnlineExtendedState getParent() {
        return parent;
    }

    /**
     * Sets the parent state in the Viterbi sequence.
     *
     * @param parent The new parent state to set.
     */
    public void setParent(OnlineExtendedState parent) {
        this.parent = parent;
    }
}

