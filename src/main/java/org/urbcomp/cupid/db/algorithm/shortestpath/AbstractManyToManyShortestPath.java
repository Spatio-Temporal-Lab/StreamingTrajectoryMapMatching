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
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner.BiDijkstraNode;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.inner.PathCache;
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

    public Map<RoadNode, Map<RoadNode, Path>> findShortestPath(
            Set<CandidatePoint> startPoints,
            Set<CandidatePoint> endPoints,
            PathCache pathCache
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
        return findManyToManyShortestPath(startNodes, endNodes, pathCache);
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

    public Map<RoadNode, Map<RoadNode, Path>> findManyToManyShortestPath(
            Set<RoadNode> startNodes,
            Set<RoadNode> endNodes,
            PathCache pathCache
    ) {
        Map<RoadNode, Map<RoadNode, Path>> results = new HashMap<>();
        for (RoadNode startNode : startNodes) {
            Map<RoadNode, Path> tmpMap = new HashMap<>();
            for (RoadNode endNode : endNodes) {
                if (!isConnected(startNode, endNode)) {
                    tmpMap.put(
                            endNode,
                            new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>())
                    );
                    continue;
                }
                // 计算最短路径并更新缓存
                Path shortestPath = findBiDirectionalShortestPath(startNode, endNode, pathCache);
                tmpMap.put(endNode, shortestPath);
            }
            results.put(startNode, tmpMap);
        }
        pathCache.updateCache();
        return results;
    }

    /**
     * 使用双向迪杰斯特拉算法查找两点之间的最短路径
     */
    private Path findBiDirectionalShortestPath(RoadNode startNode, RoadNode endNode, PathCache pathCache) {
        // 检查缓存中是否已经存在该路径
        Path cachedPath = pathCache.getPathFromCache(startNode, endNode);
        if (cachedPath != null) {
            return cachedPath; // 如果缓存中有，则直接返回
        }

        // 内部缓存，用于存储相邻节点的最短路径，避免重复计算
//        Map<RoadNode, Map<RoadNode, Path>> internalCache = new HashMap<>();

        // 创建前向和后向的优先队列
        PriorityQueue<BiDijkstraNode> forwardQueue = new PriorityQueue<>(Comparator.comparingDouble(BiDijkstraNode::getDistance));
        PriorityQueue<BiDijkstraNode> backwardQueue = new PriorityQueue<>(Comparator.comparingDouble(BiDijkstraNode::getDistance));

        // 用于存储访问的节点和距离
        Map<RoadNode, Double> forwardDistances = new HashMap<>();
        Map<RoadNode, Double> backwardDistances = new HashMap<>();

        // 前向和后向的前驱节点
        Map<RoadNode, RoadNode> forwardPrev = new HashMap<>();
        Map<RoadNode, RoadNode> backwardPrev = new HashMap<>();

        forwardDistances.put(startNode, 0.0);
        backwardDistances.put(endNode, 0.0);
        forwardQueue.add(new BiDijkstraNode(startNode, 0));
        backwardQueue.add(new BiDijkstraNode(endNode, 0));

        double shortestPathLength = Double.MAX_VALUE;
        RoadNode meetingNode = null;

        // 双向 Dijkstra 搜索
        boolean isFound = false;
//        while (!forwardQueue.isEmpty() && !backwardQueue.isEmpty()) {
        while (!forwardQueue.isEmpty() && !backwardQueue.isEmpty() && !isFound) {
            // 扩展图
            expandGraph(forwardQueue, backwardQueue, forwardDistances, backwardDistances, forwardPrev, backwardPrev);
            if (forwardQueue.isEmpty() || backwardQueue.isEmpty()) break;

            BiDijkstraNode forwardFirstNode = forwardQueue.peek();
            BiDijkstraNode backwardFirstNode = backwardQueue.peek();

            // 检查是否存在相交的路径
            double minPointDistance = forwardFirstNode.getDistance() + backwardFirstNode.getDistance();

            for (RoadNode forwardNode : forwardPrev.keySet()) {
                if (!backwardPrev.containsKey(forwardNode)) continue;

                // 找到相交路径
                double pathDistance = forwardDistances.get(forwardNode) + backwardDistances.get(forwardNode);
                if (pathDistance <= minPointDistance) {
                    shortestPathLength = pathDistance;
                    meetingNode = forwardNode;
                    isFound = true;
                    break;
                }
//                if (pathDistance < shortestPathLength) {
//                    shortestPathLength = pathDistance;
//                    meetingNode = forwardNode;
//                }
            }
        }

        // 如果找到了最短路径
        if (meetingNode != null) {
            List<RoadSegment> edgeList = generateShortestPath(meetingNode, forwardPrev, backwardPrev);
            List<SpatialPoint> points = new ArrayList<>();
            List<Integer> roadSegmentIds = new ArrayList<>();
            for (RoadSegment rs : edgeList) {
                points.addAll(rs.getPoints());
                roadSegmentIds.add(rs.getRoadSegmentId());
            }

            Path shortestPath = new Path(shortestPathLength, points, roadSegmentIds);
            pathCache.putPathInCache(startNode, endNode, shortestPath); // 将路径存入缓存
            return shortestPath;
        }

        return new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());  // 未找到路径
    }

    private void expandGraph(
            PriorityQueue<BiDijkstraNode> forwardQueue,
            PriorityQueue<BiDijkstraNode> backwardQueue,
            Map<RoadNode, Double> forwardDistances,
            Map<RoadNode, Double> backwardDistances,
            Map<RoadNode, RoadNode> forwardPrev,
            Map<RoadNode, RoadNode> backwardPrev) {

        assert !forwardQueue.isEmpty();
        assert !backwardQueue.isEmpty();

        BiDijkstraNode forwardNode = forwardQueue.peek();
        BiDijkstraNode backwardNode = backwardQueue.peek();

        BiDijkstraNode currNode;
        Map<RoadNode, Double> distances;
        Map<RoadNode, RoadNode> prev;
        PriorityQueue<BiDijkstraNode> pq;

        if (forwardNode.getDistance() < backwardNode.getDistance()) {
            distances = forwardDistances;
            prev = forwardPrev;
            pq = forwardQueue;
            currNode = forwardQueue.poll();
            while (currNode != null && currNode.getDistance() == forwardNode.getDistance()) {
                expandSingleGraph(pq, distances, prev, currNode, true);
                currNode = forwardQueue.poll();
            }
        } else if (forwardNode.getDistance() > backwardNode.getDistance()) {
            distances = backwardDistances;
            prev = backwardPrev;
            pq = backwardQueue;
            currNode = backwardQueue.poll();
            while (currNode != null && currNode.getDistance() == backwardNode.getDistance()) {
                expandSingleGraph(pq, distances, prev, currNode, false);
                currNode = backwardQueue.poll();
            }
        } else {
            currNode = forwardQueue.poll();
            while (currNode != null && currNode.getDistance() == forwardNode.getDistance()) {
                expandSingleGraph(forwardQueue, forwardDistances, forwardPrev, currNode, true);
                currNode = forwardQueue.poll();
            }
            currNode = backwardQueue.poll();
            while (currNode != null && currNode.getDistance() == backwardNode.getDistance()) {
                expandSingleGraph(backwardQueue, backwardDistances, backwardPrev, currNode, false);
                currNode = backwardQueue.poll();
            }
        }
    }

    private void expandSingleGraph(
            PriorityQueue<BiDijkstraNode> pq,
            Map<RoadNode, Double> distances,
            Map<RoadNode, RoadNode> prev,
            BiDijkstraNode currNode,
            boolean isForward) {

        RoadNode node = currNode.getNode();
        Set<RoadSegment> segments = isForward ? node.getOutgoingSegments(roadNetwork) : node.getIncomingSegments(roadNetwork);

        for (RoadSegment segment : segments) {
            RoadNode neighbor = isForward ? segment.getEndNode() : segment.getStartNode();
            double newDistance = distances.get(node) + segment.getLengthInMeter(); // TODO:距离计算

            if (newDistance < distances.getOrDefault(neighbor, Double.MAX_VALUE)) {
                distances.put(neighbor, newDistance);
                prev.put(neighbor, node);
                pq.add(new BiDijkstraNode(neighbor, newDistance));
            }
        }
    }

    private List<RoadSegment> generateShortestPath(RoadNode meetingNode,
                                                   Map<RoadNode, RoadNode> forwardPrev,
                                                   Map<RoadNode, RoadNode> backwardPrev) {
        List<RoadSegment> edgeList = new ArrayList<>();

        // 从 meetingNode 追溯到起点
        RoadNode node = meetingNode;
        while (forwardPrev.containsKey(node)) {
            RoadNode prevNode = forwardPrev.get(node);
            if (prevNode == null) break;

            // 获取从 prevNode 到 node 的 RoadSegment
            RoadSegment edge = getRoadSegment(prevNode, node); // 通过查找获取 RoadSegment
            edgeList.add(0, edge);  // 反向插入
            node = prevNode;
        }

        // 从 meetingNode 追溯到终点
        node = meetingNode;
        while (backwardPrev.containsKey(node)) {
            RoadNode nextNode = backwardPrev.get(node);
            if (nextNode == null) break;

            // 获取从 node 到 prevNode 的 RoadSegment
            RoadSegment edge = getRoadSegment(node, nextNode); // 通过查找获取 RoadSegment
            edgeList.add(edge);  // 正向插入
            node = nextNode;
        }

        return edgeList;
    }

    private RoadSegment getRoadSegment(RoadNode from, RoadNode to) {
        // 获取 from 节点的所有出边
        Set<RoadSegment> segments = from.getOutgoingSegments(roadNetwork);

        // 找到指向 to 节点的 RoadSegment
        for (RoadSegment segment : segments) {
            if (segment.getEndNode().equals(to)) {
                return segment;
            }
        }

        // 如果没有找到匹配的边，返回 null 或者抛出异常
        throw new IllegalStateException("No RoadSegment found between the nodes");
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
