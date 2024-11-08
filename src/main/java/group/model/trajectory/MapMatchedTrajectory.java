package group.model.trajectory;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import group.model.point.MapMatchedPoint;
import group.util.FeatureCollectionWithProperties;
import org.geojson.Feature;

import java.io.Serializable;
import java.util.List;

public class MapMatchedTrajectory implements Serializable {
    private final String tid;

    private final String oid;

    private final List<MapMatchedPoint> mmPtList;


    public MapMatchedTrajectory(String tid, String oid, List<MapMatchedPoint> mmPtList) {
        this.tid = tid;
        this.oid = oid;
        this.mmPtList = mmPtList;
    }

    public String getOid() {
        return oid;
    }

    public List<MapMatchedPoint> getMmPtList() {
        return mmPtList;
    }

    public String getTid() {
        return tid;
    }

    public String toGeoJSON() throws JsonProcessingException {
        FeatureCollectionWithProperties fcp = new FeatureCollectionWithProperties();
        fcp.setProperty("oid", oid);
        fcp.setProperty("tid", tid);
        for (MapMatchedPoint p : mmPtList) {
            if (p.getCandidatePoint() != null) {
                fcp.add(extractFeatureFromMatchedPoint(p));
            }
        }
        return new ObjectMapper().writeValueAsString(fcp);
    }

    /**
     * Converts the map-matched points to a GeoJSON format.
     * Optionally includes the raw point's latitude and longitude.
     *
     * @param includeRawCoords Boolean value to decide whether to include rawPoint's latitude and longitude.
     * @return A GeoJSON string representing the map-matched points.
     * @throws JsonProcessingException In case of JSON processing errors.
     */
    public String toGeoJSON(boolean includeRawCoords) throws JsonProcessingException {
        FeatureCollectionWithProperties fcp = new FeatureCollectionWithProperties();
        fcp.setProperty("oid", oid);
        fcp.setProperty("tid", tid);
        for (MapMatchedPoint p : mmPtList) {
            if (p.getCandidatePoint() != null) {
                Feature f = extractFeatureFromMatchedPoint(p);
                // If includeRawCoords is true, add rawPoint's latitude and longitude to properties.
                if (includeRawCoords) {
                    f.setProperty("rawLat", p.getRawPoint().getLat());
                    f.setProperty("rawLon", p.getRawPoint().getLng());
                }

                fcp.add(f);
            }
        }
        return new ObjectMapper().writeValueAsString(fcp);
    }

    private Feature extractFeatureFromMatchedPoint(MapMatchedPoint p) {
        Feature f = new Feature();
        f.setGeometry(
                new org.geojson.Point(
                        p.getCandidatePoint().getX(),
                        p.getCandidatePoint().getY()
                )
        );
        f.setProperty("time", p.getRawPoint().getTime().toString());
        f.setProperty("roadSegmentId", p.getCandidatePoint().getRoadSegmentId());
        f.setProperty(
                "errorDistanceInMeter",
                p.getCandidatePoint().getErrorDistanceInMeter()
        );
        f.setProperty("matchedIndex", p.getCandidatePoint().getMatchedIndex());
        f.setProperty("offsetInMeter", p.getCandidatePoint().getOffsetInMeter());
        return f;
    }
}
