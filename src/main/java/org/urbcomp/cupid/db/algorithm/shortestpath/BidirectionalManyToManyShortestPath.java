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
        return findManyToManyShortestPath(startNodes, endNodes, false);
    }

    public void getCache(){
        Map<RoadNode, Map<RoadNode, Path>> cache = resultCache.getCache();
        long totalSizeInBytes = 0;

        // 假设每个 RoadNode 大约占 32 字节
        int roadNodeSize = 32;

        // 假设每个 Path 对象的基本大小为 128 字节
        int pathBaseSize = 128;

        // 假设每个 SpatialPoint 对象大约占 24 字节
        int spatialPointSize = 24;

        // 假设每个 RoadSegment ID (int) 大约占 4 字节
        int roadSegmentIdSize = 4;

        for (Map.Entry<RoadNode, Map<RoadNode, Path>> entry : cache.entrySet()) {
            totalSizeInBytes += roadNodeSize; // 加入 key (startNode) 的大小
            for (Map.Entry<RoadNode, Path> innerEntry : entry.getValue().entrySet()) {
                totalSizeInBytes += roadNodeSize; // 加入 key (endNode) 的大小
                Path path = innerEntry.getValue();

                // 加入 Path 对象的基础大小
                totalSizeInBytes += pathBaseSize;

                // 加入 Path 中的 SpatialPoint 列表的大小
                totalSizeInBytes += path.getPoints().size() * spatialPointSize;

                // 加入 Path 中的 roadSegmentIds 列表的大小
                totalSizeInBytes += path.getRoadSegmentIds().size() * roadSegmentIdSize;
            }
        }

        double totalSizeInMB = totalSizeInBytes / (1024.0 * 1024.0);

        System.out.printf("当前缓存占用的估计内存大小: %.2f MB\n", totalSizeInMB);
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
                // 如果启用缓存，首先检查缓存中是否已经有结果
                if (useCache) {
                    Path cachedPath = resultCache.getCachedPath(startNode, endNode);
                    if (cachedPath != null) {
                        tmpMap.put(endNode, cachedPath);
                        continue;
                    }
                }

                // 如果起点和终点相同
                if (startNode.equals(endNode)) {
                    tmpMap.put(endNode, new Path(0, new ArrayList<>(), new ArrayList<>()));
                    if (useCache) {
                        resultCache.cachePath(startNode, endNode, tmpMap.get(endNode));
                    }
                    continue;
                }

                // 使用算法查找最短路径
                GraphPath<RoadNode, RoadSegment> shortestPath = algo.getPath(startNode, endNode);
                Path resultPath;

                if (shortestPath == null || shortestPath.getLength() == 0) {
                    // 没有有效路径
                    resultPath = new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
                } else {
                    // 计算路径细节
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
                // 如果启用缓存，缓存结果
                if (useCache) {
                    resultCache.cachePath(startNode, endNode, resultPath);
                }
            }
            results.put(startNode, tmpMap);
        }

        return results;
    }
}

class MapResultCache {
    private final Map<RoadNode, Map<RoadNode, Path>> cache;

    public MapResultCache() {
        this.cache = new HashMap<>();
    }

    // 获取缓存的路径
    public Path getCachedPath(RoadNode startNode, RoadNode endNode) {
        if (cache.containsKey(startNode)) {
            return cache.get(startNode).getOrDefault(endNode, null);
        }
        return null;
    }

    // 缓存路径
    public void cachePath(RoadNode startNode, RoadNode endNode, Path path) {
        cache.computeIfAbsent(startNode, k -> new HashMap<>()).put(endNode, path);
    }

    // 清空缓存
    public void clearCache() {
        cache.clear();
    }

    public Map<RoadNode, Map<RoadNode, Path>> getCache() {
        return cache;
    }




}
