package org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner;

import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;

public class BiDijkstraNode {
    private RoadNode node;
    private double distance;

    public BiDijkstraNode(RoadNode node, double distance) {
        this.node = node;
        this.distance = distance;
    }

    public BiDijkstraNode(RoadNode node) {
        this.node = node;
        this.distance = 0;
    }

    public RoadNode getNode() {
        return node;
    }

    public void setNode(RoadNode node) {
        this.node = node;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
