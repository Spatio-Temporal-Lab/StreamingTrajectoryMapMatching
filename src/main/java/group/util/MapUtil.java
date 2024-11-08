package group.util;

import group.model.Attribute;
import group.model.point.GPSPoint;
import group.model.point.SpatialPoint;

import java.util.Map;

public class MapUtil {

    static final double R = 6371000.0;
    public static boolean additionalAttributesEquals(
        Map<String, Attribute> attributeMap1,
        Map<String, Attribute> attributeMap2
    ) {
        if (attributeMap1.size() != attributeMap2.size()) {
            return false;
        }
        for (Map.Entry<String, Attribute> entry : attributeMap1.entrySet()) {
            if (!attributeMap2.containsKey(entry.getKey())) {
                return false;
            }
            if (!attributeMap2.get(entry.getKey()).equals(entry.getValue())) {
                return false;
            }
        }
        return true;
    }


    public static double calculateDirection(SpatialPoint pointA, SpatialPoint pointB) {
        double lat1 = Math.toRadians(pointA.getLat());
        double lon1 = Math.toRadians(pointA.getLng());
        double lat2 = Math.toRadians(pointB.getLat());
        double lon2 = Math.toRadians(pointB.getLng());

        double y = Math.sin(lon2 - lon1) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
        double theta = Math.atan2(y, x);
        return (Math.toDegrees(theta) + 360.0) % 360;
    }

    public static double calculateDistance(SpatialPoint pointA, SpatialPoint pointB) {
        double lat1 = Math.toRadians(pointA.getLat());
        double lon1 = Math.toRadians(pointA.getLng());
        double lat2 = Math.toRadians(pointB.getLat());
        double lon2 = Math.toRadians(pointB.getLng());

        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                        Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return R * c;
    }

    public static double calculateGPSPointVelocity(GPSPoint prev, GPSPoint curr) {
        double distance = calculateDistance(prev, curr);
        return distance / (curr.getTime().getTime() - prev.getTime().getTime());
    }
}
