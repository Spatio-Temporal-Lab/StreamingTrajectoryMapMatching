package group.model;

import java.util.Objects;

public class Attribute {
    private final String type;
    private final Object value;

    public Attribute(String type, Object value) {
        this.type = type;
        this.value = value;
    }

    public String getType() {
        return type;
    }

    public Object getValue() {
        return value;
    }

    @Override
    public String toString() {
        return type + " " + value.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Attribute attribute = (Attribute) o;
        return type.equals(attribute.type) && value.equals(attribute.value);
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, value);
    }
}
