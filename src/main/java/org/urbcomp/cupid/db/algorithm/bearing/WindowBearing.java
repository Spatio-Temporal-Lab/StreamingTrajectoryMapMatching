package org.urbcomp.cupid.db.algorithm.bearing;

import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.ArrayList;
import java.util.List;

public class WindowBearing {

    private List<GPSPoint> points;
    private List<Double> distances;
    private List<double[]> bearings;
    private double currBearing;
    private static final double WINDOWS_SIZE = 3;

    public WindowBearing(){
        this.points = new ArrayList<>();
        this.distances = new ArrayList<>();
        this.bearings = new ArrayList<>();
        this.currBearing = -1;
    }

    public void addPoint(GPSPoint point){
        if (points.isEmpty()){
            points.add(point);
            return;
        }
        double tempDis = GeoFunctions.getDistanceInM(point.getLng(), point.getLat(), points.get(points.size()-1).getLng(), points.get(points.size()-1).getLat());
        if (points.size() == WINDOWS_SIZE) {
            if (tempDis != 0){
                points.remove(0);
                distances.remove(0);
                bearings.remove(0);
            }else {
                return;
            }
        }
        if (tempDis == 0){
            return;
        }
        double[] xy = GeoFunctions.getBearingXY(points.get(points.size()-1).getLng(), points.get(points.size()-1).getLat(), point.getLng(),point.getLat());
        distances.add(tempDis);
        bearings.add(xy);
        double totalDistance = distances.stream().mapToDouble(Double::doubleValue).sum();
        double x = 0;
        double y = 0;
        for (int i = 0; i < distances.size(); i++){
            tempDis = distances.get(i);
            x += bearings.get(i)[0] * tempDis / totalDistance;
            y += bearings.get(i)[1] * tempDis / totalDistance;
        }
        currBearing = GeoFunctions.toRad(x, y);
        points.add(point);
    }

    public double getCurrBearing(){
        return currBearing;
    }


}
