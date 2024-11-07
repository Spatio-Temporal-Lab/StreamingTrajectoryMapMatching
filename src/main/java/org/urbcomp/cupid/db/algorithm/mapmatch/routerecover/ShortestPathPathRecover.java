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
package org.urbcomp.cupid.db.algorithm.mapmatch.routerecover;

import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractShortestPath;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ShortestPathPathRecover {
    private final AbstractShortestPath pathAlgo;
    private final RoadNetwork roadNetwork;

    public ShortestPathPathRecover(RoadNetwork roadNetwork, AbstractShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
    }

    public List<PathOfTrajectory> recover(MapMatchedTrajectory mmTraj) {
        List<PathOfTrajectory> results = new ArrayList<>();

        List<MapMatchedPoint> mmPoints = mmTraj.getMmPtList()
            .stream()
            .filter(o -> o.getCandidatePoint() != null)
            .collect(Collectors.toList());
        PathOfTrajectory pt = new PathOfTrajectory(mmTraj.getTid(), mmTraj.getTid());
        for (int i = 0; i < mmPoints.size() - 1; i++) {
            CandidatePoint pre = mmPoints.get(i).getCandidatePoint();
            CandidatePoint next = mmPoints.get(i + 1).getCandidatePoint();
            RoadSegment preRS = this.roadNetwork.getRoadSegmentById(pre.getRoadSegmentId());
            RoadSegment nextRS = this.roadNetwork.getRoadSegmentById(next.getRoadSegmentId());

            if (pre.getRoadSegmentId() == next.getRoadSegmentId()) {
                pt.addRoadSegmentIdIfNotEqual(pre.getRoadSegmentId());
                pt.addPointIfNotEqual(pre);
                for (int j = pre.getMatchedIndex() + 1; j <= next.getMatchedIndex(); j++) {
                    pt.addPointIfNotEqual(preRS.getPoints().get(j));
                }
                pt.addPointIfNotEqual(next);
            } else {

                pt.addRoadSegmentIdIfNotEqual(pre.getRoadSegmentId());
                pt.addPointIfNotEqual(pre);
                for (int j = pre.getMatchedIndex() + 1; j < preRS.getPoints().size(); j++) {
                    pt.addPointIfNotEqual(preRS.getPoints().get(j));
                }
                Path path = pathAlgo.findShortestPath(preRS.getEndNode(), nextRS.getStartNode());
                if (path.getLengthInMeter() == Double.MAX_VALUE) {

                    results.add(pt);
                    pt = new PathOfTrajectory(mmTraj.getTid(), mmTraj.getOid());
                } else {
                    path.getPoints().forEach(pt::addPointIfNotEqual);
                    path.getRoadSegmentIds().forEach(pt::addRoadSegmentIdIfNotEqual);

                    for (int j = 0; j <= next.getMatchedIndex(); j++) {
                        pt.addPointIfNotEqual(nextRS.getPoints().get(j));
                    }
                    pt.addPointIfNotEqual(next);
                    pt.addRoadSegmentIdIfNotEqual(next.getRoadSegmentId());
                }
            }
        }

        if (mmPoints.size() > 0) {
            CandidatePoint last = mmPoints.get(mmPoints.size() - 1).getCandidatePoint();
            pt.addPointIfNotEqual(last);
            pt.addRoadSegmentIdIfNotEqual(last.getRoadSegmentId());
            results.add(pt);
        }

        return results;
    }
}
