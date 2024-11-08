package group.algorithm.shortestpath;

import group.model.roadnetwork.RoadNetwork;
import group.util.GeoFunctions;

public class AStarShortestPath extends AbstractShortestPath {
    public AStarShortestPath(RoadNetwork roadNetwork, double searchDistance) {
        super(
            roadNetwork,
            searchDistance,
            new org.jgrapht.alg.shortestpath.AStarShortestPath<>(
                roadNetwork.getDirectedRoadGraph(),
                GeoFunctions::getDistanceInM
            )
        );
    }

    public AStarShortestPath(RoadNetwork roadNetwork) {
        super(
            roadNetwork,
            new org.jgrapht.alg.shortestpath.AStarShortestPath<>(
                roadNetwork.getDirectedRoadGraph(),
                GeoFunctions::getDistanceInM
            )
        );
    }
}
