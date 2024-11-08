package group.serializer;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import group.model.roadnetwork.RoadNetwork;

import java.io.IOException;

public class RoadNetworkDeserializer extends StdDeserializer<RoadNetwork> {
    protected RoadNetworkDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public RoadNetwork deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
        return RoadNetwork.fromGeoJSON(p.getValueAsString());
    }
}
