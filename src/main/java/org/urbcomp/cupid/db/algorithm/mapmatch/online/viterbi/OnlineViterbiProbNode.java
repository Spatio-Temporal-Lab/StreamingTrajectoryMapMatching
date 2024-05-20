package org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayListNode;

import java.util.List;

public class OnlineViterbiProbNode<T> extends TwoWayListNode<OnlineViterbiProbNode<T>> {
    public List<T> probs;

    public OnlineViterbiProbNode(List<T> probList) {
        super();
        this.probs = probList;
    }

    public OnlineViterbiProbNode(List<T> probList, OnlineViterbiProbNode<T> prev) {
        this.probs = probList;
        setNext(null);
        setPrev(prev);
    }

    public OnlineViterbiProbNode(List<T> probList, OnlineViterbiProbNode<T> prev, OnlineViterbiProbNode<T> next) {
        this.probs = probList;
        setNext(next);
        setPrev(prev);
    }

    @Override
    public String toString() {
        return "OnlineViterbiProbNode{" +
                "probs=" + probs +
                ", prev=" + prev +
                ", next=" + next +
                '}';
    }
}
