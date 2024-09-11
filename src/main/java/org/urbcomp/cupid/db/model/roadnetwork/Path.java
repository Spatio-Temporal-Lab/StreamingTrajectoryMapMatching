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
package org.urbcomp.cupid.db.model.roadnetwork;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.geojson.Feature;
import org.geojson.LngLatAlt;
import org.urbcomp.cupid.db.model.point.SpatialPoint;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * {@link Path} is different from {@link RoadSegment}.
 * It contains the points from one position to another, and specifies the cost.
 * It also contains the corresponding {@link RoadSegment} ids.
 */
public class Path {
    /**
     * distance
     */
    private double lengthInMeter;

    /**
     * the points the route crosses. This is very useful for display.
     */
    private List<SpatialPoint> points;

    /**
     * the road segment ids that this route crosses
     */
    private List<Integer> roadSegmentIds;

    public Path(double lengthInMeter, List<SpatialPoint> points, List<Integer> roadSegmentIds) {
        this.lengthInMeter = lengthInMeter;
        this.points = points;
        this.roadSegmentIds = roadSegmentIds;
    }

    public double getLengthInMeter() {
        return lengthInMeter;
    }

    public Path setLengthInMeter(double lengthInMeter) {
        this.lengthInMeter = lengthInMeter;
        return this;
    }

    public List<SpatialPoint> getPoints() {
        return points;
    }

    public Path setPoints(List<SpatialPoint> points) {
        this.points = points;
        return this;
    }

    public List<Integer> getRoadSegmentIds() {
        return roadSegmentIds;
    }

    public Path setRoadSegmentIds(List<Integer> roadSegmentIds) {
        this.roadSegmentIds = roadSegmentIds;
        return this;
    }

    @Override
    public String toString() {
        try {
            return this.toGeoJSON();
        } catch (JsonProcessingException e) {
            throw new RuntimeException(e);
        }
    }

    public Path addPath(Path path) {
        this.lengthInMeter += path.lengthInMeter;
        this.points.addAll(path.getPoints());
        this.roadSegmentIds.addAll(path.getRoadSegmentIds());
        return this;
    }

    public double calDisWeightDirection(RoadNetwork roadNetwork){
        List<Double> distances = GeoFunctions.getDistanceInMs(points);
        double totalDistance = distances.stream().mapToDouble(Double::doubleValue).sum();
        if (totalDistance == 0 && roadSegmentIds.size()==1){
            RoadSegment segment = roadNetwork.getRoadSegmentById(roadSegmentIds.get(0));
            RoadNode start = segment.getStartNode();
            RoadNode end = segment.getEndNode();
            return GeoFunctions.getBearing(start.getLng(), start.getLat(), end.getLng(), end.getLat());
        }
        double bearing = 0;
        double x = 0;
        double y = 0;
        for (int i = 1; i < points.size(); i++) {
            double[] xy = GeoFunctions.getBearingXY(points.get(i-1).getLng(), points.get(i-1).getLat(), points.get(i).getLng(), points.get(i).getLat());
            x += xy[0] * distances.get(i-1) / totalDistance;
            y += xy[1] * distances.get(i-1) / totalDistance;
        }
        return GeoFunctions.toRad(x, y);
    }


    public String toGeoJSON() throws JsonProcessingException {
        Feature f = new Feature();
        f.setProperty("lengthInMeter", lengthInMeter);
        f.setProperty("roadSegmentIds", roadSegmentIds);
        LngLatAlt[] lngLats = points.stream()
            .map(o -> new LngLatAlt(o.getLng(), o.getLat()))
            .toArray(LngLatAlt[]::new);
        f.setGeometry(new org.geojson.LineString(lngLats));
        return new ObjectMapper().writeValueAsString(f);
    }
}
