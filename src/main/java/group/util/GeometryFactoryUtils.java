package group.util;

import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.PrecisionModel;

public class GeometryFactoryUtils {
    public static GeometryFactory defaultGeometryFactory() {
        return new GeometryFactory(new PrecisionModel(), 4326);
    }
}
