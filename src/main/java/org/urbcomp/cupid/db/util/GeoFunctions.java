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
package org.urbcomp.cupid.db.util;

import org.locationtech.jts.geom.*;
import org.urbcomp.cupid.db.model.point.SpatialPoint;

import java.util.ArrayList;
import java.util.List;

public class GeoFunctions {


    public static final double EARTH_RADIUS_IN_METER = 6378137.0;


    public static final double FLATTENING = 0.00669342162296594323;


    public static double getDegreeFromM(double distance) {
        double perimeter = 2 * Math.PI * EARTH_RADIUS_IN_METER;
        double degreePerM = 360 / perimeter;
        return distance * degreePerM;
    }

    public static Envelope getExtendedBBox(SpatialPoint point, double thresholdInMeter) {
        double perimeter = 2 * Math.PI * EARTH_RADIUS_IN_METER;
        double latPerMeter = 360 / perimeter;
        double latBuffLen = thresholdInMeter * latPerMeter;
        double minLngPerMeter = 360 / (perimeter * Math.cos(Math.toRadians(point.getY())));
        double lngBuffLen = thresholdInMeter * minLngPerMeter;
        return new Envelope(
            point.getX() - lngBuffLen,
            point.getX() + lngBuffLen,
            point.getY() - latBuffLen,
            point.getY() + latBuffLen
        );
    }

    public static double getDistanceInM(double lng1, double lat1, double lng2, double lat2) {
        double radLat1 = Math.toRadians(lat1);
        double radLat2 = Math.toRadians(lat2);
        double radLatDistance = radLat1 - radLat2;
        double radLngDistance = Math.toRadians(lng1) - Math.toRadians(lng2);
        return 2 * Math.asin(
            Math.sqrt(
                Math.pow(Math.sin(radLatDistance / 2), 2) + Math.cos(radLat1) * Math.cos(radLat2)
                    * Math.pow(Math.sin(radLngDistance / 2), 2)
            )
        ) * EARTH_RADIUS_IN_METER;
    }

    public static double getDistanceInM(SpatialPoint point1, SpatialPoint point2) {
        return getDistanceInM(point1.getLng(), point1.getLat(), point2.getLng(), point2.getLat());
    }

    public static double getDistanceInM(List<SpatialPoint> points) {
        double distance = 0;
        for (int i = 1; i < points.size(); i++) {
            distance += GeoFunctions.getDistanceInM(points.get(i - 1), points.get(i));
        }
        return distance;
    }

    public static List<Double> getDistanceInMs(List<SpatialPoint> points) {
        List<Double> distances = new ArrayList<>();
        for (int i = 1; i < points.size(); i++) {
            distances.add(GeoFunctions.getDistanceInM(points.get(i - 1), points.get(i)));
        }
        return distances;
    }


    public static Envelope getBBox(List<SpatialPoint> points) {
        double minLng = Double.MAX_VALUE;
        double minLat = Double.MAX_VALUE;
        double maxLng = Double.MIN_VALUE;
        double maxLat = Double.MIN_VALUE;
        for (SpatialPoint point : points) {
            minLng = Math.min(minLng, point.getLng());
            minLat = Math.min(minLat, point.getLat());
            maxLng = Math.max(maxLng, point.getLng());
            maxLat = Math.max(maxLat, point.getLat());
        }
        return new Envelope(minLng, maxLng, minLat, maxLat);
    }

    public static Polygon bboxFromEnvelopeToPolygon(Envelope e) {
        GeometryFactory geometryFactory = GeometryFactoryUtils.defaultGeometryFactory();
        double minX = e.getMinX();
        double maxX = e.getMaxX();
        double minY = e.getMinY();
        double maxY = e.getMaxY();
        List<Point> points = new ArrayList<>(5);
        points.add(geometryFactory.createPoint(new Coordinate(minX, minY)));
        points.add(geometryFactory.createPoint(new Coordinate(maxX, minY)));
        points.add(geometryFactory.createPoint(new Coordinate(maxX, maxY)));
        points.add(geometryFactory.createPoint(new Coordinate(minX, maxY)));
        points.add(geometryFactory.createPoint(new Coordinate(minX, minY)));
        return geometryFactory.createPolygon(
            geometryFactory.createLinearRing(
                geometryFactory.createLineString(
                    points.stream().map(Point::getCoordinate).toArray(Coordinate[]::new)
                ).getCoordinateSequence()
            )
        );
    }

    public static double getBearing(double lng1, double lat1, double lng2, double lat2) {
        double dLon = Math.toRadians(lng2 - lng1);
        double y = Math.sin(dLon) * Math.cos(Math.toRadians(lat2));
        double x = Math.cos(Math.toRadians(lat1)) * Math.sin(Math.toRadians(lat2)) -
                Math.sin(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.cos(dLon);
        double bearing = Math.atan2(y, x);
        bearing = Math.toDegrees(bearing);
        bearing = (bearing + 360) % 360;
        return bearing;
    }

    public static double[] getBearingXY(double lng1, double lat1, double lng2, double lat2) {
        double dLon = Math.toRadians(lng2 - lng1);
        double y = Math.sin(dLon) * Math.cos(Math.toRadians(lat2));
        double x = Math.cos(Math.toRadians(lat1)) * Math.sin(Math.toRadians(lat2)) -
                Math.sin(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.cos(dLon);
        double[] xy = new double[2];
        xy[0] = x;
        xy[1] = y;
        return xy;
    }

    public static double getBearingDifference(SpatialPoint point1, SpatialPoint point2, SpatialPoint point3, SpatialPoint point4) {
        double bearing1 = getBearing(point1.getLng(), point1.getLat(), point2.getLng(), point2.getLat());
        double bearing2 = getBearing(point3.getLng(), point3.getLat(), point4.getLng(), point4.getLat());
        double difference = Math.abs(bearing1 - bearing2);

        if (difference > 180) {
            difference = 360 - difference;
        }
        return difference;
    }

    public static double toRad(double x, double y){
        double bearing = Math.atan2(y, x);
        bearing = Math.toDegrees(bearing);
        bearing = (bearing + 360) % 360;
        return bearing;
    }

}
