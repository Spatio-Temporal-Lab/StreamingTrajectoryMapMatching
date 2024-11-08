package group.serializer;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;
import group.model.roadnetwork.RoadSegment;

import java.io.IOException;

public class RoadSegmentSerializer extends StdSerializer<RoadSegment> {

    public RoadSegmentSerializer() {
        this(null);
    }

    protected RoadSegmentSerializer(Class<RoadSegment> t) {
        super(t);
    }

    @Override
    public void serialize(RoadSegment value, JsonGenerator gen, SerializerProvider provider)
        throws IOException {
        gen.writeString(value.toGeoJSON());
    }
}
