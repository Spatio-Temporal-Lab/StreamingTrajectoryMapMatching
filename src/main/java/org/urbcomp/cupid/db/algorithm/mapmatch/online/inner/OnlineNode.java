package org.urbcomp.cupid.db.algorithm.mapmatch.online.inner;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayListNode;

public class OnlineNode<T> extends TwoWayListNode<OnlineNode<T>> {
    public int time;
    public T state;
    public int numChild;
    public OnlineNode<T> parent;

    public OnlineNode(int time, T state, int numChild, OnlineNode<T> parent) {
        super();
        this.time = time;
        this.state = state;
        this.numChild = numChild;
        this.parent = parent;
    }

    public OnlineNode(int time, T state, int numChild, OnlineNode<T> parent, OnlineNode<T> prev) {
        this.time = time;
        this.state = state;
        this.numChild = numChild;
        this.parent = parent;
        setNext(null);
        setPrev(prev);
    }

    public OnlineNode(int time, T state, int numChild, OnlineNode<T> parent, OnlineNode<T> prev, OnlineNode<T> next) {
        this.time = time;
        this.state = state;
        this.numChild = numChild;
        this.parent = parent;
        setNext(next);
        setPrev(prev);
    }

    @Override
    public String toString() {
        return "OnlineNode{" +
                "time=" + time +
                ", state=" + state +
                ", numChild=" + numChild +
                ", parent=" + parent +
                '}';
    }
}
