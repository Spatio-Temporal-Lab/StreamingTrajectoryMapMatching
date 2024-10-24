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
    private static final int k = 1;  // 时间步窗口大小
    private static final boolean USE_CACHE = false;
    private int currentStep;

    public BidirectionalManyToManyShortestPath(RoadNetwork roadNetwork) {
        this.roadNetwork = roadNetwork;
        this.algo = new BidirectionalDijkstraShortestPath<>(roadNetwork.getDirectedRoadGraph());
        this.resultCache = new MapResultCache();
        this.currentStep = 0;
    }

    public void clearCache() {
        resultCache.clearCache();
        currentStep = 0;
    }

    public Map<RoadNode, Map<RoadNode, Path>> findShortestPath(
            Set<CandidatePoint> startPoints,
            Set<CandidatePoint> endPoints
    ) {
        // 更新当前时间步
        currentStep++;

        // 处理起始点和终点集合
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

        // 查找最短路径并更新缓存
        Map<RoadNode, Map<RoadNode, Path>> results = findManyToManyShortestPath(startNodes, endNodes, USE_CACHE);

        // 清除过期的缓存
        resultCache.evictExpiredEntries(currentStep, k);

        return results;
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
                        resultCache.updateTimeStep(startNode, endNode, currentStep);  // 更新缓存的时间步
                        continue;
                    }
                }

                // 如果起点和终点相同
                if (startNode.equals(endNode)) {
                    tmpMap.put(endNode, new Path(0, new ArrayList<>(), new ArrayList<>()));
                    if (useCache) {
                        resultCache.cachePath(startNode, endNode, tmpMap.get(endNode), currentStep);
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
                    resultCache.cachePath(startNode, endNode, resultPath, currentStep);
                }
            }
            results.put(startNode, tmpMap);
        }

        return results;
    }
}

class MapResultCache {
    private final Map<RoadNode, Map<RoadNode, CacheEntry>> cache;

    public MapResultCache() {
        this.cache = new HashMap<>();
    }

    // 获取缓存的路径
    public Path getCachedPath(RoadNode startNode, RoadNode endNode) {
        if (cache.containsKey(startNode)) {
            CacheEntry entry = cache.get(startNode).get(endNode);
            if (entry != null) {
                return entry.path;
            }
        }
        return null;
    }

    // 缓存路径并记录时间步
    public void cachePath(RoadNode startNode, RoadNode endNode, Path path, int timeStep) {
        cache.computeIfAbsent(startNode, k -> new HashMap<>())
                .put(endNode, new CacheEntry(path, timeStep));
    }

    // 更新缓存条目的时间步
    public void updateTimeStep(RoadNode startNode, RoadNode endNode, int timeStep) {
        if (cache.containsKey(startNode) && cache.get(startNode).containsKey(endNode)) {
            cache.get(startNode).get(endNode).timeStep = timeStep;
        }
    }

    // 清除过期的缓存条目
    public void evictExpiredEntries(int currentStep, int k) {
        for (Iterator<Map.Entry<RoadNode, Map<RoadNode, CacheEntry>>> it = cache.entrySet().iterator(); it.hasNext(); ) {
            Map.Entry<RoadNode, Map<RoadNode, CacheEntry>> entry = it.next();
            Map<RoadNode, CacheEntry> innerMap = entry.getValue();

            innerMap.values().removeIf(cacheEntry -> (currentStep - cacheEntry.timeStep) > k);

            // 如果内层映射为空，则移除外层映射
            if (innerMap.isEmpty()) {
                it.remove();
            }
        }
    }

    // 清空缓存
    public void clearCache() {
        cache.clear();
    }

    // 缓存条目类
    static class CacheEntry {
        Path path;
        int timeStep;

        CacheEntry(Path path, int timeStep) {
            this.path = path;
            this.timeStep = timeStep;
        }
    }
}
