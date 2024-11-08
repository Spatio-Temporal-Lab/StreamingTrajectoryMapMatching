package group.algorithm.shortestpath;

import com.github.davidmoten.guavamini.Lists;
import group.model.point.CandidatePoint;
import group.model.point.SpatialPoint;
import group.model.roadnetwork.*;
import group.util.GeoFunctions;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.interfaces.ManyToManyShortestPathsAlgorithm;

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

        ManyToManyShortestPathsAlgorithm.ManyToManyShortestPaths<RoadNode, RoadSegment> paths = algo
                .getManyToManyPaths(startNodes, endNodes);

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
                    startCandidatePoint,
                    endCandidatePoint,
                    startRoadSegment
            );
        }

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
