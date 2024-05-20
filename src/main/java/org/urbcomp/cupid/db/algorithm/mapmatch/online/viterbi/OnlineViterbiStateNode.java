package org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayListNode;

import java.util.List;

public class OnlineViterbiStateNode<T> extends TwoWayListNode<OnlineViterbiStateNode<T>> {
    public List<T> states;

    public OnlineViterbiStateNode(List<T> stateList) {
        super();
        this.states = stateList;
    }

    public OnlineViterbiStateNode(List<T> stateList, OnlineViterbiStateNode prev) {
        this.states = stateList;
        setNext(null);
        setPrev(prev);
    }

    public OnlineViterbiStateNode(List<T> stateList, OnlineViterbiStateNode prev, OnlineViterbiStateNode next) {
        this.states = stateList;
        setNext(next);
        setPrev(prev);
    }

    @Override
    public String toString() {
        return "OnlineViterbiStateNode{" +
                "states=" + states +
                ", prev=" + prev +
                ", next=" + next +
                '}';
    }
}
