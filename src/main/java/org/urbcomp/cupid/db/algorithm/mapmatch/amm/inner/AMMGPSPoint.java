package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;

import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.SpatialPoint;

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
