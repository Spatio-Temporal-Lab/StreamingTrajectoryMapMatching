package group.serializer;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import group.model.roadnetwork.RoadSegment;

import java.io.IOException;

public class RoadSegmentDeserializer extends StdDeserializer<RoadSegment> {
    protected RoadSegmentDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public RoadSegment deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
        return RoadSegment.fromGeoJSON(p.getValueAsString());
    }
}
