package group.algorithm.mapmatch.amm.inner;

import group.model.point.GPSPoint;

import java.sql.Timestamp;

public class AMMGPSPoint extends GPSPoint {
    public PointsSet parent;
    public AMMGPSPoint(Timestamp time, double lng, double lat) {
        super(time, lng, lat);
    }

    public long getTimestamp(){
        return getTime().getTime();
    }

    public double getLatitude() {
        return getLatitude();
    }

    public double getLongitude() {
        return getLongitude();
    }


}
