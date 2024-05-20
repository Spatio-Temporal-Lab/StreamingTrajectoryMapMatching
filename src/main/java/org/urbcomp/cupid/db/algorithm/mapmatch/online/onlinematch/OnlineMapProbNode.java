package org.urbcomp.cupid.db.algorithm.mapmatch.online.onlinematch;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayListNode;

import java.util.Map;

public class OnlineMapProbNode<K, V> extends TwoWayListNode<OnlineMapProbNode<K, V>> {
    public Map<K, V> probs;

    public OnlineMapProbNode(Map<K, V> probsMap) {
        super();
        this.probs = probsMap;
    }

    public OnlineMapProbNode(Map<K, V> probsMap, OnlineMapProbNode<K, V> prev) {
        this.probs = probsMap;
        setNext(null);
        setPrev(prev);
    }

    public OnlineMapProbNode(Map<K, V> probsMap, OnlineMapProbNode<K, V> prev, OnlineMapProbNode<K, V> next) {
        this.probs = probsMap;
        setNext(next);
        setPrev(prev);
    }

    @Override
    public String toString() {
        return "OnlineMapProbNode{" +
                "probs=" + probs +
                ", prev=" + prev +
                ", next=" + next +
                '}';
    }
}
