package group.algorithm.shortestpath;

import group.model.roadnetwork.RoadNetwork;
import org.jgrapht.alg.shortestpath.BidirectionalDijkstraShortestPath;

public class BiDijkstraShortestPath extends AbstractShortestPath {
    public BiDijkstraShortestPath(RoadNetwork roadNetwork, double searchDistance) {
        super(
            roadNetwork,
            searchDistance,
            new BidirectionalDijkstraShortestPath<>(roadNetwork.getDirectedRoadGraph())
        );
    }

    public BiDijkstraShortestPath(RoadNetwork roadNetwork) {
        super(
            roadNetwork,
            new BidirectionalDijkstraShortestPath<>(roadNetwork.getDirectedRoadGraph())
        );
    }


}
