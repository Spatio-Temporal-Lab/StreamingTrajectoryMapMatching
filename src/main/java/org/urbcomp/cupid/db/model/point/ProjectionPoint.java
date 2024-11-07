package org.urbcomp.cupid.db.model.point;

import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.List;

public class ProjectionPoint extends SpatialPoint{

    private final double errorDistanceInMeter;

    private final double rate;

    private final int matchedIndex;


    public ProjectionPoint(
            SpatialPoint point,
            double errorDistance,
            double rate,
            int matchedIndex
    ) {
        super(point.getLng(), point.getLat());
        this.errorDistanceInMeter = errorDistance;
        this.rate = rate;
        this.matchedIndex = matchedIndex;
    }

    public double getErrorDistanceInMeter() {
        return errorDistanceInMeter;
    }

    public double getRate() {
        return rate;
    }

    public int getMatchedIndex() {
        return matchedIndex;
    }


    public static ProjectionPoint calProjection(
            SpatialPoint pt,
            List<SpatialPoint> points,
            int start,
            int end
    ) {
        if (end - start == 1) {
            return projectPtToSegment(points.get(start), points.get(end), pt, start);
        }
        int mid = (start + end) / 2;
        ProjectionPoint projectionPoint = projectPtToSegment(
                points.get(start),
                points.get(mid),
                pt,
                -1 // this value is not used
        );
        double rate = projectionPoint.getRate();
        if (rate > 1.0) {
            start = mid;
        } else {
            end = mid;
        }
        return calProjection(pt, points, start, end);
    }

    private static ProjectionPoint projectPtToSegment(
            SpatialPoint startPt,
            SpatialPoint endPt,
            SpatialPoint pt,
            int matchedIndex
    ) {
        double abAngle = bearing(startPt, endPt);
        double atAngle = bearing(startPt, pt);
        double abLength = GeoFunctions.getDistanceInM(startPt, endPt);
        double atLength = GeoFunctions.getDistanceInM(startPt, pt);
        double deltaAngle = atAngle - abAngle;
        double metersAlong = atLength * Math.cos(Math.toRadians(deltaAngle));
        double rate;
        SpatialPoint projection;
        if (abLength == 0.0) {
            rate = 0.0;
        } else {
            rate = metersAlong / abLength;
        }
        if (rate > 1.0) {
            projection = new SpatialPoint(endPt.getLng(), endPt.getLat());
        } else if (rate < 0) {
            projection = new SpatialPoint(startPt.getLng(), startPt.getLat());
        } else {
            projection = calLocAlongLine(startPt, endPt, rate);
        }
        double dist = GeoFunctions.getDistanceInM(pt, projection);
        return new ProjectionPoint(projection, dist, rate, matchedIndex);
    }

    private static double bearing(SpatialPoint startPt, SpatialPoint endPt) {
        double ptALatRad = Math.toRadians(startPt.getLat());
        double ptALngRad = Math.toRadians(startPt.getLng());
        double ptBLatRad = Math.toRadians(endPt.getLat());
        double ptBLngRad = Math.toRadians(endPt.getLng());
        double y = Math.sin(ptBLngRad - ptALngRad) * Math.cos(ptBLatRad);
        double x = Math.cos(ptALatRad) * Math.sin(ptBLatRad) - Math.sin(ptALatRad) * Math.cos(
                ptBLatRad
        ) * Math.cos(ptBLngRad - ptALngRad);
        double bearingRad = Math.atan2(y, x);
        return (Math.toDegrees(bearingRad) + 360.0) % 360.0;
    }

    private static SpatialPoint calLocAlongLine(
            SpatialPoint startPt,
            SpatialPoint endPt,
            double rate
    ) {
        double lat = startPt.getLat() + rate * (endPt.getLat() - startPt.getLat());
        double lng = startPt.getLng() + rate * (endPt.getLng() - startPt.getLng());
        return new SpatialPoint(lng, lat);
    }
}
