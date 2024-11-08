package group.model.point;

import com.github.davidmoten.rtree.Entry;
import com.github.davidmoten.rtree.geometry.Geometries;
import com.github.davidmoten.rtree.geometry.Rectangle;
import group.util.GeoFunctions;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import group.model.roadnetwork.RoadNetwork;
import group.model.roadnetwork.RoadSegment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CandidatePoint extends SpatialPoint {
    /**
     * matched road segment id
     */
    private final int roadSegmentId;
    /**
     * the distance from the start point of the matched road segment, meter
     */
    private final double offsetInMeter;

    /**
     * the matched position index in the road segment geo
     */
    private final int matchedIndex;

    /**
     * the distance between the matched point and the gps point
     */
    private final double errorDistanceInMeter;

    /**
     * if skip map matching and ignore the result
     */
    private boolean skip = false;

    private int index = -1;

    public boolean isSkip() {
        return skip;
    }

    public void setSkip(boolean skip) {
        this.skip = skip;
    }

    public int getRoadSegmentId() {
        return roadSegmentId;
    }

    public double getOffsetInMeter() {
        return offsetInMeter;
    }

    public int getMatchedIndex() {
        return matchedIndex;
    }

    public double getErrorDistanceInMeter() {
        return errorDistanceInMeter;
    }

    public CandidatePoint() {
        super(0,0);
        this.roadSegmentId = 0;
        this.matchedIndex = 0;
        this.errorDistanceInMeter = 0;
        this.offsetInMeter = 0;
    }

    public CandidatePoint(
        SpatialPoint matchedPoint,
        RoadSegment roadSegment,
        int matchedIndex,
        double errorDistanceInMeter
    ) {
        super(matchedPoint.getLng(), matchedPoint.getLat());
        this.roadSegmentId = roadSegment.getRoadSegmentId();
        this.matchedIndex = matchedIndex;
        this.errorDistanceInMeter = errorDistanceInMeter;
        this.offsetInMeter = calOffsetInMeter(roadSegment, matchedIndex);
    }

    public CandidatePoint(double lng, double lat, int roadSegmentId, int matchedIndex, double errorDistance, double offsetInMeter) {
        super(lng, lat);
        this.roadSegmentId = roadSegmentId;
        this.matchedIndex = matchedIndex;
        this.errorDistanceInMeter = errorDistance;
        this.offsetInMeter = offsetInMeter;
    }

    private double calOffsetInMeter(RoadSegment roadSegment, int matchedIndex) {
        double offset = 0;

        List<SpatialPoint> points = roadSegment.getPoints();
        for (int i = 0; i < matchedIndex; i++) {
            offset += GeoFunctions.getDistanceInM(points.get(i), points.get(i + 1));
        }

        offset += GeoFunctions.getDistanceInM(
            new SpatialPoint(roadSegment.getPoints().get(matchedIndex).getCoordinate()),
            this
        );

        return offset;
    }

    @Override
    public String toString() {
        return this.roadSegmentId
            + "|"
            + this.offsetInMeter
            + "|"
            + this.matchedIndex
            + "|"
            + this.errorDistanceInMeter
            + "|"
            + this.getX() + " " + this.getY();
    }

    @Override
    public int hashCode() {
        return super.hashCode();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        CandidatePoint other = (CandidatePoint) o;
        return this.roadSegmentId == other.getRoadSegmentId()
            && offsetInMeter == other.offsetInMeter
            && matchedIndex == other.matchedIndex
            && errorDistanceInMeter == other.errorDistanceInMeter
            && super.equals(other);
    }

    @Override
    public boolean equals(Geometry g) {
        if (this == g) return true;
        if (g == null || getClass() != g.getClass()) return false;
        CandidatePoint other = (CandidatePoint) g;
        return Math.abs(roadSegmentId) == Math.abs(other.getRoadSegmentId())
            && errorDistanceInMeter == errorDistanceInMeter
            && super.equals(g);
    }


    public static CandidatePoint getNearestCandidatePoint(
        SpatialPoint pt,
        RoadNetwork roadNetwork,
        double dist
    ) {
        List<CandidatePoint> candidates = getCandidatePoint(pt, roadNetwork, dist, -1);
        if (candidates.size() != 0) {
            return Collections.min(
                candidates,
                Comparator.comparingDouble(CandidatePoint::getErrorDistanceInMeter)
            );
        } else {
            return null;
        }
    }


    public static List<CandidatePoint> getCandidatePoint(
        SpatialPoint pt,
        RoadNetwork roadNetwork,
        double dist,
        int index
    ) {
        Envelope bbox = GeoFunctions.getExtendedBBox(pt, dist);
        Rectangle rec = Geometries.rectangleGeographic(
            bbox.getMinX(),
            bbox.getMinY(),
            bbox.getMaxX(),
            bbox.getMaxY()
        );
        Iterable<RoadSegment> roadSegmentIterable = roadNetwork.getRoadRTree()
            .search(rec)
            .map(Entry::value)
            .toBlocking()
            .toIterable();
        List<CandidatePoint> result = new ArrayList<>();
        roadSegmentIterable.forEach(rs -> {
            CandidatePoint candiPt = calCandidatePoint(pt, rs);
            if (candiPt.errorDistanceInMeter <= dist) {
                candiPt.setIndex(index);
                result.add(candiPt);
            }
        });
        return result;
    }


    public static CandidatePoint calCandidatePoint(SpatialPoint rawPoint, RoadSegment rs) {
        List<SpatialPoint> points = rs.getPoints();
        ProjectionPoint projectionPoint = ProjectionPoint.calProjection(
            rawPoint,
            points,
            0,
            points.size() - 1
        );
        return new CandidatePoint(
            projectionPoint,
            rs,
            projectionPoint.getMatchedIndex(),
            projectionPoint.getErrorDistanceInMeter()
        );
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }
}
