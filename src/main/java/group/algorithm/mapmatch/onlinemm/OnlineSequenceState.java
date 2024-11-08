package group.algorithm.mapmatch.onlinemm;

import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;
import group.algorithm.mapmatch.tihmm.inner.SequenceState;

public class OnlineSequenceState extends SequenceState {

    public int time;

    public OnlineSequenceState(CandidatePoint state, GPSPoint observation) {
        super(state, observation);
    }

    public OnlineSequenceState(CandidatePoint state, GPSPoint observation, int time) {
        super(state, observation);
        this.time = time;
    }
}
