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
package org.urbcomp.cupid.db.algorithm.shortestpath;

import com.github.davidmoten.guavamini.Lists;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.interfaces.ShortestPathAlgorithm;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.*;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.*;

public abstract class AbstractShortestPath {
    private final ShortestPathAlgorithm<RoadNode, RoadSegment> algo;
    private final RoadNetwork roadNetwork;
    /**
     * note that the start and end points may be mot exactly on the road segments,
     * the searchDistance defines the scope to find the start or end road segments.
     * unit: meter
     */
    private double searchDistance = 300;

    public AbstractShortestPath(
        RoadNetwork roadNetwork,
        double searchDistance,
        ShortestPathAlgorithm<RoadNode, RoadSegment> algo
    ) {
        this.roadNetwork = roadNetwork;
        this.searchDistance = searchDistance;
        this.algo = algo;
    }

    public AbstractShortestPath(
        RoadNetwork roadNetwork,
        ShortestPathAlgorithm<RoadNode, RoadSegment> algo
    ) {
        this.roadNetwork = roadNetwork;
        this.algo = algo;
    }


    public Path findShortestPath(SpatialPoint startPoint, SpatialPoint endPoint) {
        if (startPoint.equals(endPoint)) {
            return new Path(0, Arrays.asList(startPoint, endPoint), new ArrayList<>());
        }
        CandidatePoint startCandidatePoint = CandidatePoint.getNearestCandidatePoint(
            startPoint,
            roadNetwork,
            searchDistance
        );
        CandidatePoint endCandidatePoint = CandidatePoint.getNearestCandidatePoint(
            endPoint,
            roadNetwork,
            searchDistance
        );

        RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
            startCandidatePoint.getRoadSegmentId()
        );
        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
            endCandidatePoint.getRoadSegmentId()
        );

        if (startRoadSegment.getRoadSegmentId() == -endRoadSegment.getRoadSegmentId()) {
            endCandidatePoint = reverseCandidatePoint(
                endCandidatePoint,
                endRoadSegment,
                startRoadSegment
            );
            endRoadSegment = startRoadSegment;
        }

        if (startRoadSegment.getRoadSegmentId() == endRoadSegment.getRoadSegmentId()
            && startCandidatePoint.getOffsetInMeter() <= endCandidatePoint.getOffsetInMeter()) {
            return getPathInSameRoadSegmentInOrder(
                startPoint,
                startCandidatePoint,
                endPoint,
                endCandidatePoint,
                startRoadSegment
            );
        }

        if (startRoadSegment.getRoadSegmentId() == endRoadSegment.getRoadSegmentId()
            && startCandidatePoint.getOffsetInMeter() > endCandidatePoint.getOffsetInMeter()
            && startRoadSegment.getDirection() == RoadSegmentDirection.DUAL) {
            return getPathInSameRoadSegmentReverse(
                startPoint,
                startCandidatePoint,
                endPoint,
                endCandidatePoint,
                startRoadSegment
            );
        }


        List<Path> paths = new ArrayList<>();
        paths.add(
            getPathByEndNodeToStartNode(
                startPoint,
                startCandidatePoint,
                startRoadSegment,
                endPoint,
                endCandidatePoint,
                endRoadSegment
            )
        );
        if (startRoadSegment.getDirection() == RoadSegmentDirection.DUAL) {
            RoadSegment reverseStartRoadSegment = roadNetwork.getRoadSegmentById(
                -startRoadSegment.getRoadSegmentId()
            );
            CandidatePoint reverseStartCandidatePoint = reverseCandidatePoint(
                startCandidatePoint,
                startRoadSegment,
                reverseStartRoadSegment
            );
            paths.add(
                getPathByEndNodeToStartNode(
                    startPoint,
                    reverseStartCandidatePoint,
                    reverseStartRoadSegment,
                    endPoint,
                    endCandidatePoint,
                    endRoadSegment
                )
            );
            if (endRoadSegment.getDirection() == RoadSegmentDirection.DUAL) {
                RoadSegment reverseEndRoadSegment = roadNetwork.getRoadSegmentById(
                    -endRoadSegment.getRoadSegmentId()
                );
                CandidatePoint reverseEndCandidatePoint = reverseCandidatePoint(
                    endCandidatePoint,
                    endRoadSegment,
                    reverseEndRoadSegment
                );
                paths.add(
                    getPathByEndNodeToStartNode(
                        startPoint,
                        reverseStartCandidatePoint,
                        reverseStartRoadSegment,
                        endPoint,
                        reverseEndCandidatePoint,
                        reverseEndRoadSegment
                    )
                );
            }
        }
        if (endRoadSegment.getDirection() == RoadSegmentDirection.DUAL) {
            RoadSegment reverseEndRoadSegment = roadNetwork.getRoadSegmentById(
                -endRoadSegment.getRoadSegmentId()
            );
            CandidatePoint reverseEndCandidatePoint = reverseCandidatePoint(
                endCandidatePoint,
                endRoadSegment,
                reverseEndRoadSegment
            );
            paths.add(
                getPathByEndNodeToStartNode(
                    startPoint,
                    startCandidatePoint,
                    startRoadSegment,
                    endPoint,
                    reverseEndCandidatePoint,
                    reverseEndRoadSegment
                )
            );
        }
        Path path = Collections.min(paths, Comparator.comparingDouble(Path::getLengthInMeter));
        if (path.getLengthInMeter() != Double.MAX_VALUE) {
            return path;
        } else {
            return new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
        }
    }

    public Path findShortestPath(RoadNode startNode, RoadNode endNode) {
        GraphPath<RoadNode, RoadSegment> shortestPath = algo.getPath(startNode, endNode);
        if (shortestPath == null) {
            return new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
        } else {
            List<SpatialPoint> points = new ArrayList<>();
            double length = 0;
            List<Integer> roadSegmentIds = new ArrayList<>();
            for (RoadSegment rs : shortestPath.getEdgeList()) {
                length += rs.getLengthInMeter();
                points.addAll(rs.getPoints());
                roadSegmentIds.add(rs.getRoadSegmentId());
            }
            return new Path(length, points, roadSegmentIds);
        }
    }

    private Path getPathByEndNodeToStartNode(
        SpatialPoint startPoint,
        CandidatePoint startCandidatePoint,
        RoadSegment startRoadSegment,
        SpatialPoint endPoint,
        CandidatePoint endCandidatePoint,
        RoadSegment endRoadSegment
    ) {
        Path result = getSubPathFromStartPoint(startPoint, startCandidatePoint, startRoadSegment);
        result.addPath(
            findShortestPath(startRoadSegment.getEndNode(), endRoadSegment.getStartNode())
        );
        result.addPath(getSubPathToEndPoint(endPoint, endCandidatePoint, endRoadSegment));
        return result;
    }

    private Path getPathInSameRoadSegmentInOrder(
        SpatialPoint startPoint,
        CandidatePoint startCandidatePoint,
        SpatialPoint endPoint,
        CandidatePoint endCandidatePoint,
        RoadSegment roadSegment
    ) {
        List<SpatialPoint> points = Lists.newArrayList(startPoint, startCandidatePoint);
        for (int i = startCandidatePoint.getMatchedIndex() + 1; i <= endCandidatePoint
            .getMatchedIndex(); i++) {
            points.add(roadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
        points.add(endPoint);
        return new Path(
            GeoFunctions.getDistanceInM(points),
            points,
            Collections.singletonList(roadSegment.getRoadSegmentId())
        );
    }

    private Path getPathInSameRoadSegmentReverse(
        SpatialPoint startPoint,
        CandidatePoint startCandidatePoint,
        SpatialPoint endPoint,
        CandidatePoint endCandidatePoint,
        RoadSegment roadSegment
    ) {
        List<SpatialPoint> points = Lists.newArrayList(startPoint, startCandidatePoint);
        for (int i = startCandidatePoint.getMatchedIndex(); i > endCandidatePoint
            .getMatchedIndex(); i--) {
            points.add(roadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
        points.add(endPoint);
        return new Path(
            GeoFunctions.getDistanceInM(points),
            points,
            Collections.singletonList(roadSegment.getRoadSegmentId())
        );
    }

    private CandidatePoint reverseCandidatePoint(
        CandidatePoint candidatePoint,
        RoadSegment rsOld,
        RoadSegment rsNew
    ) {
        return new CandidatePoint(
            candidatePoint,
            rsNew,
            rsOld.getPoints().size() - 2 - candidatePoint.getMatchedIndex(),
            candidatePoint.getErrorDistanceInMeter()
        );
    }

    private Path getSubPathFromStartPoint(
        SpatialPoint startPoint,
        CandidatePoint startCandidatePoint,
        RoadSegment startRoadSegment
    ) {
        List<SpatialPoint> points = Lists.newArrayList(startPoint, startCandidatePoint);
        for (int i = startCandidatePoint.getMatchedIndex() + 1; i < startRoadSegment.getPoints()
            .size(); i++) {
            points.add(startRoadSegment.getPoints().get(i));
        }
        return new Path(
            GeoFunctions.getDistanceInM(points),
            points,
            Lists.newArrayList(startRoadSegment.getRoadSegmentId())
        );
    }

    private Path getSubPathToEndPoint(
        SpatialPoint endPoint,
        CandidatePoint endCandidatePoint,
        RoadSegment endRoadSegment
    ) {
        List<SpatialPoint> points = new ArrayList<>();
        for (int i = 0; i < endCandidatePoint.getMatchedIndex(); i++) {
            points.add(endRoadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
        points.add(endPoint);
        return new Path(
            GeoFunctions.getDistanceInM(points),
            points,
            Lists.newArrayList(endRoadSegment.getRoadSegmentId())
        );
    }
}
