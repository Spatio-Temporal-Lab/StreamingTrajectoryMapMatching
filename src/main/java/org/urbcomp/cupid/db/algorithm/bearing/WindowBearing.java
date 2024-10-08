package org.urbcomp.cupid.db.algorithm.bearing;

import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.ArrayList;
import java.util.List;

public class WindowBearing {

    private List<GPSPoint> points;
    private List<Double> distances;
    private List<Double> bearings;
    private List<Double> change;
    private double currBearing;
    private static final double WINDOWS_SIZE = 8;

    public WindowBearing() {
        this.points = new ArrayList<>();
        this.distances = new ArrayList<>();
        this.bearings = new ArrayList<>();
        this.change = new ArrayList<>();
        this.currBearing = -1;
    }

    public void addPoint(GPSPoint point) {
        if (points.isEmpty()) {
            points.add(point);
            return;
        }
        double tempDis = GeoFunctions.getDistanceInM(point.getLng(), point.getLat(), points.get(points.size() - 1).getLng(), points.get(points.size() - 1).getLat());
        if (points.size() == WINDOWS_SIZE) {
            if (tempDis != 0) {
                points.remove(0);
                distances.remove(0);
                bearings.remove(0);
                change.remove(0);
            } else {
                return;
            }
        }
        if (tempDis == 0) {
            return;
        }
        currBearing = GeoFunctions.getBearing(points.get(points.size() - 1).getLng(), points.get(points.size() - 1).getLat(), point.getLng(), point.getLat());
        distances.add(tempDis);
        bearings.add(currBearing);
        if (bearings.size() >= 2) {
            int size = bearings.size();
            double bearDiff = Math.abs(bearings.get(size - 2) - currBearing);
            if (bearDiff > 180) {
                bearDiff = 360 - bearDiff;
            }
            change.add(bearDiff);
        }
        points.add(point);
    }

    public double getCurrBearing() {
        return currBearing;
    }

    public boolean getChange() {
        if (change.isEmpty()) {
            return false;
        }
        double changeScore = change.stream().mapToDouble(Double::doubleValue).map(o -> o / 180).sum();
        return changeScore <= 0.2;
    }

    public double getChangeScore() {
        return change.stream().mapToDouble(Double::doubleValue).map(o -> o / 180).sum();
    }


}
