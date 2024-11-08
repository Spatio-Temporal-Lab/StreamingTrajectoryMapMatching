package group.model.roadnetwork;

import org.jgrapht.graph.AbstractBaseGraph;
import org.jgrapht.graph.DefaultGraphType;
import org.jgrapht.util.SupplierUtil;

import java.util.HashSet;
import java.util.Set;

public class RoadGraph extends AbstractBaseGraph<RoadNode, RoadSegment> {
    public RoadGraph(boolean directed) {
        super(
            null,
            SupplierUtil.createSupplier(RoadSegment.class),
            new DefaultGraphType.Builder(directed, !directed).weighted(true)
                .allowMultipleEdges(true)
                .allowSelfLoops(true)
                .build()
        );
    }

    public void addEdge(RoadSegment roadSegment) {
        addVertex(roadSegment.getStartNode());
        addVertex(roadSegment.getEndNode());
        addEdge(roadSegment.getStartNode(), roadSegment.getEndNode(), roadSegment);
        setEdgeWeight(roadSegment, roadSegment.getLengthInMeter());
    }

    public static boolean areEdgesAdjacent(RoadSegment edge1, RoadSegment edge2) {
        RoadNode startNode1 = edge1.getStartNode();
        RoadNode endNode1 = edge1.getEndNode();
        RoadNode startNode2 = edge2.getStartNode();
        RoadNode endNode2 = edge2.getEndNode();

        // Check if they share any node
        return startNode1.equals(startNode2) || startNode1.equals(endNode2) ||
                endNode1.equals(startNode2) || endNode1.equals(endNode2);
    }


    public Set<RoadSegment> getOneHopReachableEdges(RoadSegment edge) {
        RoadNode endNode = edge.getEndNode();
        Set<RoadSegment> reachableEdges = new HashSet<>(this.outgoingEdgesOf(endNode));
        reachableEdges.remove(edge);

        return reachableEdges;
    }



}
