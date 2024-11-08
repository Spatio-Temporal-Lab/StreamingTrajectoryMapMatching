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
import org.jgrapht.alg.interfaces.ManyToManyShortestPathsAlgorithm;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.*;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.*;

public class AbstractManyToManyShortestPath {
    private final ManyToManyShortestPathsAlgorithm<RoadNode, RoadSegment> algo;
    private final RoadNetwork roadNetwork;

    public AbstractManyToManyShortestPath(
            RoadNetwork roadNetwork,
            ManyToManyShortestPathsAlgorithm<RoadNode, RoadSegment> algo
    ) {
        this.roadNetwork = roadNetwork;
        this.algo = algo;
    }

    public Map<RoadNode, Map<RoadNode, Path>> findShortestPath(
            Set<CandidatePoint> startPoints,
            Set<CandidatePoint> endPoints
    ) {
        // 记录候选点所在路段
        Set<RoadNode> startNodes = new HashSet<>();
        Set<RoadNode> endNodes = new HashSet<>();
        for (CandidatePoint startPt : startPoints) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    startPt.getRoadSegmentId()
            );
            // 对于起始点，获取其路段的终点
            startNodes.add(startRoadSegment.getEndNode());
        }
        for (CandidatePoint endPt : endPoints) {
            RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(endPt.getRoadSegmentId());
            // 对于结束点，获取其路段的始点
            endNodes.add(endRoadSegment.getStartNode());
        }
        return findManyToManyShortestPath(startNodes, endNodes);
    }

    public Path findShortestPathBetweenCandidates(CandidatePoint startCandidatePoint, CandidatePoint endCandidatePoint) {
        Set<CandidatePoint> startPoints = new HashSet<>();
        Set<CandidatePoint> endPoints = new HashSet<>();
        startPoints.add(startCandidatePoint);
        endPoints.add(endCandidatePoint);

        Map<RoadNode, Map<RoadNode, Path>> shortestPaths = findShortestPath(startPoints, endPoints);

        RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(startCandidatePoint.getRoadSegmentId());
        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(endCandidatePoint.getRoadSegmentId());

        RoadNode startNode = startRoadSegment.getEndNode();
        RoadNode endNode = endRoadSegment.getStartNode();

        if (shortestPaths.containsKey(startNode) && shortestPaths.get(startNode).containsKey(endNode)) {
            Path path = shortestPaths.get(startNode).get(endNode);
            return getCompletePath(startCandidatePoint, endCandidatePoint, path);
        } else {
            return new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
        }
    }

    public Map<RoadNode, Map<RoadNode, Path>> findManyToManyShortestPath(
            Set<RoadNode> startNodes,
            Set<RoadNode> endNodes
    ) {
        // 返回任意两点之间的最短路径
        ManyToManyShortestPathsAlgorithm.ManyToManyShortestPaths<RoadNode, RoadSegment> paths = algo
                .getManyToManyPaths(startNodes, endNodes);

        Map<RoadNode, Map<RoadNode, Path>> results = new HashMap<>();
        for (RoadNode startNode : startNodes) {
            Map<RoadNode, Path> tmpMap = new HashMap<>();
            for (RoadNode endNode : endNodes) {
                if (!isConnected(startNode, endNode)) {
                    // 如果两个点不连通则直接返回 Path
                    tmpMap.put(
                            endNode,
                            new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>())
                    );
                    continue;
                }
                GraphPath<RoadNode, RoadSegment> shortestPath = paths.getPath(startNode, endNode);
                if (shortestPath.getLength() == 0 && !(startNode.getLat() == endNode.getLat() && startNode.getLng() == endNode.getLng())) {
                    tmpMap.put(
                            endNode,
                            new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>())
                    );
                } else {
                    List<SpatialPoint> points = new ArrayList<>();
                    double length = 0;
                    List<Integer> roadSegmentIds = new ArrayList<>();
                    for (RoadSegment rs : shortestPath.getEdgeList()) {
                        length += rs.getLengthInMeter();
                        points.addAll(rs.getPoints());
                        roadSegmentIds.add(rs.getRoadSegmentId());
                    }
                    tmpMap.put(endNode, new Path(length, points, roadSegmentIds));

                }
            }
            results.put(startNode, tmpMap);
        }
        return results;

    }

    public boolean isConnected(RoadNode startNode, RoadNode endNode) {
        return algo.getPath(startNode, endNode) != null;
    }


    public Path getCompletePath(
            CandidatePoint startCandidatePoint,
            CandidatePoint endCandidatePoint,
            Path path
    ) {
        RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                startCandidatePoint.getRoadSegmentId()
        );
        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(
                endCandidatePoint.getRoadSegmentId()
        );

        // 两个点在同一条双向路段上，并且方向相反
        if (startRoadSegment.getRoadSegmentId() == -endRoadSegment.getRoadSegmentId()) {
            endCandidatePoint = reverseCandidatePoint(
                    endCandidatePoint,
                    endRoadSegment,
                    startRoadSegment
            );
            endRoadSegment = startRoadSegment;
        }
        // 两个点在同一条路段上, 并且出发点在前
        if (startRoadSegment.getRoadSegmentId() == endRoadSegment.getRoadSegmentId()
                && startCandidatePoint.getOffsetInMeter() <= endCandidatePoint.getOffsetInMeter()) {
            return getPathInSameRoadSegmentInOrder(
                    startCandidatePoint,
                    endCandidatePoint,
                    startRoadSegment
            );
        }
        // 两个点在同一条双向路段上，并且出发点在后
        if (startRoadSegment.getRoadSegmentId() == endRoadSegment.getRoadSegmentId()
                && startCandidatePoint.getOffsetInMeter() > endCandidatePoint.getOffsetInMeter()
                && startRoadSegment.getDirection() == RoadSegmentDirection.DUAL) {
            return getPathInSameRoadSegmentReverse(
                    startCandidatePoint,
                    endCandidatePoint,
                    startRoadSegment
            );
        } else {
            Path result = getSubPathFromStartPoint(startCandidatePoint, startRoadSegment);
            result.addPath(path);
//            if (startCandidatePoint.getRoadSegmentId() == 61448){
//                System.out.println("end path " + endCandidatePoint.getRoadSegmentId()+ " " + getSubPathToEndPoint(endCandidatePoint, endRoadSegment));
//            }
            result.addPath(getSubPathToEndPoint(endCandidatePoint, endRoadSegment));
            return result;

        }

    }

    private Path getSubPathFromStartPoint(
            CandidatePoint startCandidatePoint,
            RoadSegment startRoadSegment
    ) {
        List<SpatialPoint> points = new ArrayList<>();
        points.add(startCandidatePoint);
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
            CandidatePoint endCandidatePoint,
            RoadSegment endRoadSegment
    ) {
        List<SpatialPoint> points = new ArrayList<>();
        for (int i = 0; i <= endCandidatePoint.getMatchedIndex(); i++) {
            points.add(endRoadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
        return new Path(
                GeoFunctions.getDistanceInM(points),
                points,
                Lists.newArrayList(endRoadSegment.getRoadSegmentId())
        );
    }

    private Path getPathInSameRoadSegmentInOrder(
            CandidatePoint startCandidatePoint,
            CandidatePoint endCandidatePoint,
            RoadSegment roadSegment
    ) {
        List<SpatialPoint> points = new ArrayList<>();
        points.add(startCandidatePoint);
        for (int i = startCandidatePoint.getMatchedIndex() + 1; i <= endCandidatePoint
                .getMatchedIndex(); i++) {
            points.add(roadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
        return new Path(
                GeoFunctions.getDistanceInM(points),
                points,
                Collections.singletonList(roadSegment.getRoadSegmentId())
        );
    }

    private Path getPathInSameRoadSegmentReverse(
            CandidatePoint startCandidatePoint,
            CandidatePoint endCandidatePoint,
            RoadSegment roadSegment
    ) {
        List<SpatialPoint> points = new ArrayList<>();
        points.add(startCandidatePoint);
        for (int i = startCandidatePoint.getMatchedIndex(); i > endCandidatePoint
                .getMatchedIndex(); i--) {
            points.add(roadSegment.getPoints().get(i));
        }
        points.add(endCandidatePoint);
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

}
