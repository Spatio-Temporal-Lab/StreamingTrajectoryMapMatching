package group.algorithm.mapmatch.tihmm.inner;

import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;


public class SequenceState {

    private final CandidatePoint state;

    private final GPSPoint observation;

    public SequenceState(CandidatePoint state, GPSPoint observation) {
        this.state = state;
        this.observation = observation;
    }

    public CandidatePoint getState() {
        return state;
    }

    public GPSPoint getObservation() {
        return observation;
    }

}
