package group.serializer;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;
import group.model.roadnetwork.RoadNetwork;

import java.io.IOException;

public class RoadNetworkSerializer extends StdSerializer<RoadNetwork> {

    public RoadNetworkSerializer() {
        this(null);
    }

    protected RoadNetworkSerializer(Class<RoadNetwork> t) {
        super(t);
    }

    @Override
    public void serialize(RoadNetwork value, JsonGenerator gen, SerializerProvider provider)
        throws IOException {
        gen.writeString(value.toGeoJSON());
    }
}
