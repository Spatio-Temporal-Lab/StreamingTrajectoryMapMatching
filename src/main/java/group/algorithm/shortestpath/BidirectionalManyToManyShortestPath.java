package group.algorithm.shortestpath;

import group.model.point.CandidatePoint;
import group.model.point.SpatialPoint;
import group.model.roadnetwork.Path;
import group.model.roadnetwork.RoadNetwork;
import group.model.roadnetwork.RoadNode;
import group.model.roadnetwork.RoadSegment;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.BidirectionalDijkstraShortestPath;

import java.util.*;

public class BidirectionalManyToManyShortestPath {
    private final BidirectionalDijkstraShortestPath<RoadNode, RoadSegment> algo;
    private final RoadNetwork roadNetwork;
    private final MapResultCache resultCache;
    private static final int k = 5;
    private static final boolean USE_CACHE = true;
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

        currentStep++;

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


        Map<RoadNode, Map<RoadNode, Path>> results = findManyToManyShortestPath(startNodes, endNodes, USE_CACHE);


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

                if (useCache) {
                    Path cachedPath = resultCache.getCachedPath(startNode, endNode);
                    if (cachedPath != null) {
                        tmpMap.put(endNode, cachedPath);
                        resultCache.updateTimeStep(startNode, endNode, currentStep);
                        continue;
                    }
                }


                if (startNode.equals(endNode)) {
                    tmpMap.put(endNode, new Path(0, new ArrayList<>(), new ArrayList<>()));
                    if (useCache) {
                        resultCache.cachePath(startNode, endNode, tmpMap.get(endNode), currentStep);
                    }
                    continue;
                }


                GraphPath<RoadNode, RoadSegment> shortestPath = algo.getPath(startNode, endNode);
                Path resultPath;

                if (shortestPath == null || shortestPath.getLength() == 0) {

                    resultPath = new Path(Double.MAX_VALUE, new ArrayList<>(), new ArrayList<>());
                } else {

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


    public Path getCachedPath(RoadNode startNode, RoadNode endNode) {
        if (cache.containsKey(startNode)) {
            CacheEntry entry = cache.get(startNode).get(endNode);
            if (entry != null) {
                return entry.path;
            }
        }
        return null;
    }


    public void cachePath(RoadNode startNode, RoadNode endNode, Path path, int timeStep) {
        cache.computeIfAbsent(startNode, k -> new HashMap<>())
                .put(endNode, new CacheEntry(path, timeStep));
    }

    public void updateTimeStep(RoadNode startNode, RoadNode endNode, int timeStep) {
        if (cache.containsKey(startNode) && cache.get(startNode).containsKey(endNode)) {
            cache.get(startNode).get(endNode).timeStep = timeStep;
        }
    }


    public void evictExpiredEntries(int currentStep, int k) {
        for (Iterator<Map.Entry<RoadNode, Map<RoadNode, CacheEntry>>> it = cache.entrySet().iterator(); it.hasNext(); ) {
            Map.Entry<RoadNode, Map<RoadNode, CacheEntry>> entry = it.next();
            Map<RoadNode, CacheEntry> innerMap = entry.getValue();

            innerMap.values().removeIf(cacheEntry -> (currentStep - cacheEntry.timeStep) > k);

            if (innerMap.isEmpty()) {
                it.remove();
            }
        }
    }


    public void clearCache() {
        cache.clear();
    }


    static class CacheEntry {
        Path path;
        int timeStep;

        CacheEntry(Path path, int timeStep) {
            this.path = path;
            this.timeStep = timeStep;
        }
    }
}
