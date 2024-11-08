package group.model.point;

import group.util.MapUtil;
import group.model.Attribute;

import java.sql.Timestamp;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * GPS Point class
 */
public class GPSPoint extends SpatialPoint {
    /**
     * the timestamp of the GPS Point
     */
    private final Timestamp time;

    private Map<String, Attribute> attributes = new HashMap<>();

    public GPSPoint(Timestamp time, double lng, double lat) {
        super(lng, lat);
        this.time = time;
    }

    public GPSPoint(Timestamp time, double lng, double lat, Map<String, Attribute> attributes) {
        super(lng, lat);
        this.time = time;
        this.attributes = attributes;
    }

    /**
     * get the timestamp of the GPS point
     * @return timestamp
     */
    public Timestamp getTime() {
        return time;
    }

    public Map<String, Attribute> getAttributes() {
        return attributes;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        return Objects.equals(this.time, ((GPSPoint) o).time)
            && MapUtil.additionalAttributesEquals(attributes, ((GPSPoint) o).attributes)
            && super.equalsExact(((GPSPoint) o));
    }

    @Override
    public int hashCode() {
        return super.hashCode() + (time == null ? 0 : time.hashCode());
    }
}
