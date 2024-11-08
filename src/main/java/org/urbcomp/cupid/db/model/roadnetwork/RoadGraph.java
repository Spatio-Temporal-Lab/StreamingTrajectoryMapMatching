/* 
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.model.roadnetwork;

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

    /**
     * 获取某条路段的所有邻接边（不包括自身）
     * @param edge 图中的边，代表一条路段
     * @return 对应路段的所有邻接边
     */
    public Set<RoadSegment> getOneHopReachableEdges(RoadSegment edge) {
        RoadNode endNode = edge.getEndNode();
        Set<RoadSegment> reachableEdges = new HashSet<>(this.outgoingEdgesOf(endNode));
        reachableEdges.remove(edge);

        return reachableEdges;
    }



}
