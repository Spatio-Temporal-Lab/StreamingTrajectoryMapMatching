package org.urbcomp.cupid.db.algorithm.mapmatch.online.onlinematch;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayListNode;

import java.util.Map;

public class OnlineMapStateNode<K, V> extends TwoWayListNode<OnlineMapStateNode<K, V>> {

    public Map<K, V> states;

    public OnlineMapStateNode(Map<K, V> statesMap) {
        super();
        this.states = statesMap;
    }

    public OnlineMapStateNode(Map<K, V> statesMap, OnlineMapStateNode<K, V> prev) {
        this.states = statesMap;
        setNext(null);
        setPrev(prev);
    }

    public OnlineMapStateNode(Map<K, V> statesMap, OnlineMapStateNode<K, V> prev, OnlineMapStateNode<K, V> next) {
        this.states = statesMap;
        setNext(next);
        setPrev(prev);
    }

    @Override
    public String toString() {
        return "OnlineMapStateNode{" +
                "states=" + states +
                ", prev=" + prev +
                ", next=" + next +
                '}';
    }
}
