package group.algorithm.mapmatch.tihmm.inner;

import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;


public class ExtendedState {

    private final CandidatePoint state;

    private final ExtendedState backPointer;

    private final GPSPoint observation;

    public ExtendedState(CandidatePoint state, ExtendedState backPointer, GPSPoint observation) {
        this.state = state;
        this.backPointer = backPointer;
        this.observation = observation;
    }

    public CandidatePoint getState() {
        return state;
    }

    public ExtendedState getBackPointer() {
        return backPointer;
    }

    public GPSPoint getObservation() {
        return observation;
    }
}
