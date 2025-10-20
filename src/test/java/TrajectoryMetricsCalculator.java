import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;

public class TrajectoryMetricsCalculator {

    // 地球平均半径（单位：米）
    private final double EARTH_RADIUS_METERS = 6371000;

    public double totalDistance = 0.0;

    public int validPointsCount = 0;

    /**
     * 计算地图匹配轨迹中，原始点与匹配点之间的平均距离。
     *
     * @param trajectory 地图匹配后的轨迹对象
     * @return 平均距离（单位：米）。如果轨迹中没有有效的匹配点，则返回 0.0。
     */
    public void calculateAverageMatchingDistance(MapMatchedTrajectory trajectory) {
        if (trajectory == null || trajectory.getMmPtList() == null || trajectory.getMmPtList().isEmpty()) {
            return;
        }

        for (MapMatchedPoint mmPoint : trajectory.getMmPtList()) {
            // 确保原始点和候选匹配点都存在
            if (mmPoint != null && mmPoint.getRawPoint() != null && mmPoint.getCandidatePoint() != null) {
                GPSPoint rawPoint = mmPoint.getRawPoint();
                CandidatePoint candidatePoint = mmPoint.getCandidatePoint();

                // 计算两个点之间的距离
                double distance = calculateDistanceInMeters(
                        rawPoint.getLat(), rawPoint.getLng(),
                        candidatePoint.getLat(), candidatePoint.getLng()
                );

                totalDistance += distance;
                validPointsCount++;
            }
        }
    }

    /**
     * 使用 Haversine 公式计算两个地理坐标点之间的距离。
     *
     * @param lat1 点1的纬度
     * @param lon1 点1的经度
     * @param lat2 点2的纬度
     * @param lon2 点2的经度
     * @return 两点之间的距离（单位：米）
     */
    private double calculateDistanceInMeters(double lat1, double lon1, double lat2, double lon2) {
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);

        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return EARTH_RADIUS_METERS * c;

    }
}