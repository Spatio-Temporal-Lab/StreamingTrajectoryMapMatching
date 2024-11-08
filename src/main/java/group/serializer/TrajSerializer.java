package group.serializer;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;
import group.model.trajectory.Trajectory;

import java.io.IOException;

public class TrajSerializer extends StdSerializer<Trajectory> {

    public TrajSerializer() {
        this(null);
    }

    protected TrajSerializer(Class<Trajectory> t) {
        super(t);
    }

    @Override
    public void serialize(Trajectory traj, JsonGenerator gen, SerializerProvider provider)
        throws IOException {
        gen.writeString(traj.toGeoJSON());
    }
}
