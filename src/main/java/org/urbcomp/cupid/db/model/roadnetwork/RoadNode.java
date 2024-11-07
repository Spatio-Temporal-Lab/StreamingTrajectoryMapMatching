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

import org.jgrapht.alg.interfaces.ManyToManyShortestPathsAlgorithm;
import org.locationtech.jts.geom.Coordinate;
import org.urbcomp.cupid.db.model.point.SpatialPoint;

import java.util.ArrayList;
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
