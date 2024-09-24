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

import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.BidirectionalDijkstraShortestPath;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;

import java.util.*;

public class BidirectionalManyToManyShortestPath {
    private final BidirectionalDijkstraShortestPath<org.urbcomp.cupid.db.model.roadnetwork.RoadNode, org.urbcomp.cupid.db.model.roadnetwork.RoadSegment> algo;
    private final RoadNetwork roadNetwork;
    private final MapResultCache resultCache;

    public BidirectionalManyToManyShortestPath(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.algo = new BidirectionalDijkstraShortestPath<>(roadNetwork.getDirectedRoadGraph());
        this.resultCache = new MapResultCache();
    }


    public Map<RoadNode, Map<RoadNode, Path>> findShortestPath(
            Set<CandidatePoint> startPoints,
            Set<CandidatePoint> endPoints
    ) {

        Set<RoadNode> startNodes = new HashSet<>();
        Set<RoadNode> endNodes = new HashSet<>();
        for (CandidatePoint startPt : startPoints) {
            RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(
                    startPt.getRoadSegmentId()
            );
            startNodes.add(startRoadSegment.getEndNode());
        }
        for (CandidatePoint endPt : endPoints) {
            RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(endPt.getRoadSegmentId());
            endNodes.add(endRoadSegment.getStartNode());
        }
        return findManyToManyShortestPath(startNodes, endNodes, true);
//        return findManyToManyShortestPath(startNodes, endNodes);
    }

    public Map<RoadNode, Map<RoadNode, Path>> findManyToManyShortestPath(
            Set<RoadNode> startNodes,
            Set<RoadNode> endNodes,
            boolean useCache
    ) {
        Map<RoadNode, Map<RoadNode, Path>> results = new HashMap<>();

        for (RoadNode startNode : startNodes) {
            Map<RoadNode, Path> tmpMap = new HashMap<>();

            for (RoadNode endNode : endNodes) {

                if (resultCache.getPreviousResult().containsKey(startNode) &&
                        resultCache.getPreviousResult().get(startNode).containsKey(endNode)) {
                    tmpMap.put(endNode, resultCache.getPreviousResult().get(startNode).get(endNode));
                    continue;
                }

                // Check if the start and end nodes are the same
                if (startNode.equals(endNode)) {
                    // Handle case where start and end nodes are the same
                    tmpMap.put(endNode, new Path(0, new ArrayList<>(), new ArrayList<>()));
                    continue;
                }

                // Check if there is a path between startNode and endNode
                GraphPath<RoadNode, RoadSegment> shortestPath = algo.getPath(startNode, endNode);
                Path resultPath;

                if (shortestPath == null || shortestPath.getLength() == 0) {
                    // No valid path exists
                    resultPath = new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
                } else {
                    // Calculate path details
                    List<SpatialPoint> points = new ArrayList<>();
                    double length = 0;
                    List<Integer> roadSegmentIds = new ArrayList<>();

                    for (RoadSegment rs : shortestPath.getEdgeList()) {
                        length += rs.getLengthInMeter();
                        points.addAll(rs.getPoints());
                        roadSegmentIds.add(rs.getRoadSegmentId());
                    }

                    resultPath = new Path(length, points, roadSegmentIds);
                }
                tmpMap.put(endNode, resultPath);
            }

            // Update the current result cache with the temporary results
            resultCache.updateCurrentResult(startNode, tmpMap);
            results.put(startNode, tmpMap);
        }

        return results;
    }

    public Map<RoadNode, Map<RoadNode, Path>> findManyToManyShortestPath(
            Set<RoadNode> startNodes,
            Set<RoadNode> endNodes
    ) {
        Map<RoadNode, Map<RoadNode, Path>> results = new HashMap<>();

        for (RoadNode startNode : startNodes) {
            Map<RoadNode, Path> tmpMap = new HashMap<>();

            for (RoadNode endNode : endNodes) {
                // Check if the start and end nodes are the same
                if (startNode.equals(endNode)) {
                    // Handle case where start and end nodes are the same
                    tmpMap.put(endNode, new Path(0, new ArrayList<>(), new ArrayList<>()));
                    continue;
                }

                // Check if there is a path between startNode and endNode
                GraphPath<RoadNode, RoadSegment> shortestPath = algo.getPath(startNode, endNode);
                Path resultPath;

                if (shortestPath == null || shortestPath.getLength() == 0) {
                    // No valid path exists
                    resultPath = new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
                } else {
                    // Calculate path details
                    List<SpatialPoint> points = new ArrayList<>();
                    double length = 0;
                    List<Integer> roadSegmentIds = new ArrayList<>();

                    for (RoadSegment rs : shortestPath.getEdgeList()) {
                        length += rs.getLengthInMeter();
                        points.addAll(rs.getPoints());
                        roadSegmentIds.add(rs.getRoadSegmentId());
                    }

                    resultPath = new Path(length, points, roadSegmentIds);
                }
                tmpMap.put(endNode, resultPath);
            }
            results.put(startNode, tmpMap);
        }

        return results;
    }

}

class MapResultCache {
    private final Map<RoadNode, Map<RoadNode, Path>> currentResult;
    private final Map<RoadNode, Map<RoadNode, Path>> previousResult;

    public MapResultCache() {
        this.currentResult = new HashMap<>();
        this.previousResult = new HashMap<>();
    }

    // 更新当前结果
    public void updateCurrentResult(RoadNode startNode, Map<RoadNode, Path> paths) {
        currentResult.put(startNode, paths);
    }

    // 清空当前结果，并将其赋值给上一次结果
    public void refreshResults() {
        previousResult.clear();
        previousResult.putAll(currentResult);
        currentResult.clear();
    }

    // 获取当前结果
    public Map<RoadNode, Map<RoadNode, Path>> getCurrentResult() {
        return currentResult;
    }

    // 获取上一次结果
    public Map<RoadNode, Map<RoadNode, Path>> getPreviousResult() {
        return previousResult;
    }
}
