package group.util;

import static group.util.GeoFunctions.EARTH_RADIUS_IN_METER;
import static group.util.GeoFunctions.FLATTENING;


public class CoordTransformUtils {

    private static final double X_PI = Math.PI * 3000.0 / 180.0;

    public static double[] bd09Towgs84(double lng, double lat) {
        double[] gcj = bd09Togcj02(lng, lat);
        return gcj02Towgs84(gcj[0], gcj[1]);
    }

    public static double[] wgs84Tobd09(double lng, double lat) {
        double[] gcj = wgs84Togcj02(lng, lat);
        return gcj02Tobd09(gcj[0], gcj[1]);
    }

    public static double[] gcj02Tobd09(double lng, double lat) {
        double z = Math.sqrt(lng * lng + lat * lat) + 0.00002 * Math.sin(lat * X_PI);
        double theta = Math.atan2(lat, lng) + 0.000003 * Math.cos(lng * X_PI);
        double bdLng = z * Math.cos(theta) + 0.0065;
        double bdLat = z * Math.sin(theta) + 0.006;
        return new double[] { bdLng, bdLat };
    }


    public static double[] bd09Togcj02(double lat, double lng) {
        double x = lat - 0.0065;
        double y = lng - 0.006;
        double z = Math.sqrt(x * x + y * y) - 0.00002 * Math.sin(y * X_PI);
        double theta = Math.atan2(y, x) - 0.000003 * Math.cos(x * X_PI);
        double ggLng = z * Math.cos(theta);
        double ggLat = z * Math.sin(theta);
        return new double[] { ggLng, ggLat };
    }


    public static double[] wgs84Togcj02(double lng, double lat) {
        if (isOutOfChina(lng, lat)) {
            return new double[] { lng, lat };
        }
        double[] delta = calDelta(lng, lat);
        return new double[] { lng + delta[0], lat + delta[1] };
    }


    public static double[] gcj02Towgs84(double lng, double lat) {
        if (isOutOfChina(lng, lat)) {
            return new double[] { lng, lat };
        }

        double[] delta = calDelta(lng, lat);
        return new double[] { lng - delta[0], lat - delta[1] };
    }


    private static double[] calDelta(double lng, double lat) {
        double[] delta = new double[2];
        double dLat = calDeltaLat(lng - 105.0, lat - 35.0);
        double dlng = calDeltaLng(lng - 105.0, lat - 35.0);
        double radlat = lat / 180.0 * Math.PI;
        double magic = Math.sin(radlat);
        magic = 1 - FLATTENING * magic * magic;
        double sqrtmagic = Math.sqrt(magic);
        delta[0] = (dlng * 180.0) / (EARTH_RADIUS_IN_METER / sqrtmagic * Math.cos(radlat)
            * Math.PI);
        delta[1] = (dLat * 180.0) / ((EARTH_RADIUS_IN_METER * (1 - FLATTENING)) / (magic
            * sqrtmagic) * Math.PI);
        return delta;
    }


    private static double calDeltaLat(double lng, double lat) {
        double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * Math
            .sqrt(Math.abs(lng));
        ret += (20.0 * Math.sin(6.0 * lng * Math.PI) + 20.0 * Math.sin(2.0 * lng * Math.PI)) * 2.0
            / 3.0;
        ret += (20.0 * Math.sin(lat * Math.PI) + 40.0 * Math.sin(lat / 3.0 * Math.PI)) * 2.0 / 3.0;
        ret += (160.0 * Math.sin(lat / 12.0 * Math.PI) + 320 * Math.sin(lat * Math.PI / 30.0)) * 2.0
            / 3.0;
        return ret;
    }


    private static double calDeltaLng(double lng, double lat) {
        double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * Math.sqrt(
            Math.abs(lng)
        );
        ret += (20.0 * Math.sin(6.0 * lng * Math.PI) + 20.0 * Math.sin(2.0 * lng * Math.PI)) * 2.0
            / 3.0;
        ret += (20.0 * Math.sin(lng * Math.PI) + 40.0 * Math.sin(lng / 3.0 * Math.PI)) * 2.0 / 3.0;
        ret += (150.0 * Math.sin(lng / 12.0 * Math.PI) + 300.0 * Math.sin(lng / 30.0 * Math.PI))
            * 2.0 / 3.0;
        return ret;
    }


    private static boolean isOutOfChina(double lng, double lat) {
        return lng < 72.004 || lng > 137.8347 || lat < 0.8293 || lat > 55.8271;
    }
}
