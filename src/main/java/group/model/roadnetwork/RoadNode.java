package group.model.roadnetwork;

import group.model.point.SpatialPoint;
import org.locationtech.jts.geom.Coordinate;

import java.util.Set;

public class RoadNode extends SpatialPoint {
    private final int nodeId;

    public RoadNode(int nodeId, double lng, double lat) {
        super(lng, lat);
        this.nodeId = nodeId;
    }

    public RoadNode(int nodeId, Coordinate coordinate) {
        super(coordinate);
        this.nodeId = nodeId;
    }

    public Set<RoadSegment> getOutgoingSegments(RoadNetwork roadNetwork) {

        RoadGraph graph = roadNetwork.getDirectedRoadGraph();
        return graph.outgoingEdgesOf(this);
    }

    public Set<RoadSegment> getIncomingSegments(RoadNetwork roadNetwork) {

        RoadGraph graph = roadNetwork.getDirectedRoadGraph();
        return graph.incomingEdgesOf(this);
    }

    public int getNodeId() {
        return this.nodeId;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        RoadNode rn = (RoadNode) o;
        return nodeId == rn.nodeId;
    }

    @Override
    public int hashCode() {
        return nodeId;
    }
}
