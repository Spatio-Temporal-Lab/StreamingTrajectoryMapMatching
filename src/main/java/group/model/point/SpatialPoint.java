package group.model.point;

import group.util.GeometryFactoryUtils;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.impl.CoordinateArraySequence;

public class SpatialPoint extends Point {
    public SpatialPoint(double lng, double lat) {
        super(
            new CoordinateArraySequence(new Coordinate[] { new Coordinate(lng, lat) }),
            GeometryFactoryUtils.defaultGeometryFactory()
        );
    }

    public SpatialPoint(Coordinate coordinate) {
        this(coordinate.x, coordinate.y);
    }

    public double getLng() {
        return this.getX();
    }

    public double getLat() {
        return this.getY();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        return super.equalsExact(((SpatialPoint) o));
    }
}
