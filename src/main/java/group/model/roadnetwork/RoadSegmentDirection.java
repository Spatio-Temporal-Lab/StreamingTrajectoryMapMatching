package group.model.roadnetwork;

public enum RoadSegmentDirection {

    UNDEFINED(-1),
    // dual way, i.e., bi-directional
    DUAL(1),
    // forward way
    FORWARD(2),
    // backward way
    BACKWARD(3);

    private final int value;

    RoadSegmentDirection(int value) {
        this.value = value;
    }

    public static RoadSegmentDirection valueOf(int value) {
        for (RoadSegmentDirection type : values()) {
            if (type.value() == value) {
                return type;
            }
        }
        return null;
    }

    public int value() {
        return this.value;
    }
}
