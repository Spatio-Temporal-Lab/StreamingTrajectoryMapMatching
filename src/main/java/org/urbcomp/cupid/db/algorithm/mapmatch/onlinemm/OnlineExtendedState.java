package org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm;

import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.ExtendedState;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;

public class OnlineExtendedState extends ExtendedState {

    private int time;
    private int numOfChild;
    private int numOfState;
    private OnlineExtendedState parent;
    /**
     * 构造函数
     *
     * @param state       candidate point
     * @param backPointer 向后的指针
     * @param observation 原始point
     */
    public OnlineExtendedState(CandidatePoint state, ExtendedState backPointer, GPSPoint observation) {
        super(state, backPointer, observation);
    }

    public OnlineExtendedState(CandidatePoint state, ExtendedState backPointer, GPSPoint observation,
                               int time, int numOfChild, int numOfState, OnlineExtendedState parent) {
        super(state, backPointer, observation);
        this.time = time;
        this.numOfChild = numOfChild;
        this.numOfState = numOfState;
        this.parent = parent;
    }

    public int getTime() {
        return time;
    }

    public void setTime(int time) {
        this.time = time;
    }

    public int getNumOfChild() {
        return numOfChild;
    }

    public void setNumOfChild(int numOfChild) {
        this.numOfChild = numOfChild;
    }

    public int getNumOfState() {
        return numOfState;
    }

    public void setNumOfState(int numOfState) {
        this.numOfState = numOfState;
    }

    public OnlineExtendedState getParent() {
        return parent;
    }

    public void setParent(OnlineExtendedState parent) {
        this.parent = parent;
    }
}
