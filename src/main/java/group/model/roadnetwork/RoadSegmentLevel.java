package group.model.roadnetwork;

public enum RoadSegmentLevel {

    UNDEFINED(-1),

    ELEVATED_ROAD(0),

    HIGH_WAY_ROAD(1),

    NATIONAL_ROAD(2),

    PROVINCIAL_ROAD(3),

    COUNTRY_ROAD(4),

    MAIN_ROAD(5),

    URBAN_ROAD(6),

    DOWNTOWN_ROAD(7),

    RESIDENTIAL_ROAD(8),

    SIDEWALK_ROAD(9);

    private final int value;

    RoadSegmentLevel(int value) {
        this.value = value;
    }

    public static RoadSegmentLevel valueOf(int value) {
        for (RoadSegmentLevel type : values()) {
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
