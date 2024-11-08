package group.model.point;

import java.sql.Timestamp;

public class MapMatchedPoint implements java.io.Serializable {

    private final GPSPoint rawPoint;

    private final CandidatePoint candidatePoint;

    public MapMatchedPoint(GPSPoint rawPoint, CandidatePoint candidatePoint) {
        this.rawPoint = rawPoint;
        this.candidatePoint = candidatePoint;
    }

    public GPSPoint getRawPoint() {
        return rawPoint;
    }

    public Timestamp getTime() {
        return this.rawPoint.getTime();
    }

    public CandidatePoint getCandidatePoint() {
        return candidatePoint;
    }


    @Override
    public String toString(){
        return candidatePoint.getLng() + "," + candidatePoint.getLat() + "  " + rawPoint.getTime();
    }

    @Override
    public int hashCode() {
        String code = String.format(
            "%d%d",
            this.getCandidatePoint().hashCode(),
            this.getTime().hashCode()
        );
        char[] charArr = code.toCharArray();
        int hashcode = 0;
        for (char c : charArr) {
            hashcode = hashcode * 131 + c;
        }
        return hashcode;
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof MapMatchedPoint
            && ((MapMatchedPoint) o).getCandidatePoint().equals(getCandidatePoint())
            && ((MapMatchedPoint) o).getTime().equals(getTime());
    }
}
