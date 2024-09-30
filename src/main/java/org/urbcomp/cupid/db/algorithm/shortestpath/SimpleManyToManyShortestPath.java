package org.urbcomp.cupid.db.algorithm.shortestpath;

import org.jgrapht.alg.shortestpath.DijkstraManyToManyShortestPaths;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;

public class SimpleManyToManyShortestPath extends AbstractManyToManyShortestPath {
    public SimpleManyToManyShortestPath(RoadNetwork roadNetwork) {
        super(roadNetwork, new DijkstraManyToManyShortestPaths<>(roadNetwork.getDirectedRoadGraph()));
    }
}
