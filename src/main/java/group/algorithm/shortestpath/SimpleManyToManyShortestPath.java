package group.algorithm.shortestpath;

import group.model.roadnetwork.RoadNetwork;
import org.jgrapht.alg.shortestpath.DijkstraManyToManyShortestPaths;

public class SimpleManyToManyShortestPath extends AbstractManyToManyShortestPath {
    public SimpleManyToManyShortestPath(RoadNetwork roadNetwork) {
        super(roadNetwork, new DijkstraManyToManyShortestPaths<>(roadNetwork.getDirectedRoadGraph()));
    }
}
