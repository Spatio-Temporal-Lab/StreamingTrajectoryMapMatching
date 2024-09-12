package org.urbcomp.cupid.db.util;

import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;

/**
 * 计算位置、方向、速度误差
 */
public class CalculateErrors {

    // 位置误差：匹配点和GPS观测点之间的直线距离
    public static double calculatePositionError(GPSPoint gpsPoint, CandidatePoint matchedPoint) {
        return calculateDistance(gpsPoint, matchedPoint);
    }

    private static double calculateDistance(SpatialPoint point1, SpatialPoint point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // 方向误差：相邻两个匹配点方向与两个观测点方向之间的差异
    public static double calculateDirectionError(GPSPoint gpsPoint1, GPSPoint gpsPoint2, CandidatePoint matchedPoint1, CandidatePoint matchedPoint2) {
        double gpsDirection = calculateBearing(gpsPoint1, gpsPoint2);
        double matchedDirection = calculateBearing(matchedPoint1, matchedPoint2);

        double directionError = Math.abs(gpsDirection - matchedDirection);
        return Math.min(directionError, 360 - directionError); // 处理角度差大于180度的情况
    }

    // 计算方向角
    private static double calculateBearing(SpatialPoint point1, SpatialPoint point2) {
        double deltaX = point2.getX() - point1.getX();
        double deltaY = point2.getY() - point1.getY();

        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    // 速度误差：相邻两个匹配点速度与两个观测点速度之间的差异
    public static double calculateSpeedError(GPSPoint gpsPoint1, GPSPoint gpsPoint2, CandidatePoint matchedPoint1, CandidatePoint matchedPoint2) {
        double timeInterval = Math.abs(gpsPoint1.getTime() - gpsPoint2.getTime());

        double gpsDistance = calculateDistance(gpsPoint1, gpsPoint2);
        double matchedDistance = calculateDistance(matchedPoint1, matchedPoint2);

        double gpsSpeed = gpsDistance / timeInterval;
        double matchedSpeed = matchedDistance / timeInterval;

        return Math.abs(gpsSpeed - matchedSpeed);
    }

}
