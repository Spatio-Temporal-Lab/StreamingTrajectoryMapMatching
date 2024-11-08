package group.serializer;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import group.model.trajectory.Trajectory;

import java.io.IOException;

public class TrajDeserializer extends StdDeserializer<Trajectory> {

    protected TrajDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public Trajectory deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
        return Trajectory.fromGeoJSON(p.getValueAsString());
    }
}
