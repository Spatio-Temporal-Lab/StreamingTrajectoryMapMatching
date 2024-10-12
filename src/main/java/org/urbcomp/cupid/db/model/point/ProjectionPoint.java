package org.urbcomp.cupid.db.model.point;

import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.List;

// gps点在路段上的投影点
public class ProjectionPoint extends SpatialPoint{
    /**
     * 误差距离，单位为m
     */
    private final double errorDistanceInMeter;
    /**
     * 投影的斜率
     */
    private final double rate;

    private final int matchedIndex;

    /**
     * 构造函数
     *
     * @param point         映射点
     * @param errorDistance 匹配误差
     * @param rate          斜率
     */
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

    /**
     * 二分法快速查找映射点
     *
     * @param pt     原始点
     * @param points 映射路段的point list
     * @param start  起始index
     * @param end    结束index
     * @return 投影点, 投影点对应的point list 的index
     */
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

    /**
     * 将点投影到线， 如果点在 ab组成这个线段的左边， 投影为a， 如果在右边，投影为b， 如果在中间，按照rate求出对应投影点的经纬度
     *
     * @param startPt 点 一个rs上的起始点
     * @param endPt   点 一个rs上的终点
     * @param pt      待投影的点
     * @return 投影到segment上的点
     */
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

    /**
     * 计算两个点之间的倾斜角度
     *
     * @param startPt 点
     * @param endPt   点
     * @return 倾斜角度
     */
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

    /**
     * 根据rate 求出投影点的经纬度
     *
     * @param startPt 起始点
     * @param endPt   终点
     * @param rate    a到投影点距离 / ab距离
     * @return 投影点
     */
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
