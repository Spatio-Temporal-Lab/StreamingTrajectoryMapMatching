package org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm;

import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;

public class OnlineSequenceState extends SequenceState {

    public int time;
    /**
     * 构造函数
     *
     * @param state       原始point的candidate
     * @param observation 原始point
     */
    public OnlineSequenceState(CandidatePoint state, GPSPoint observation) {
        super(state, observation);
    }

    public OnlineSequenceState(CandidatePoint state, GPSPoint observation, int time) {
        super(state, observation);
        this.time = time;
    }
}
