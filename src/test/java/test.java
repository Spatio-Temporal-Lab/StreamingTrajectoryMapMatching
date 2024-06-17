import com.fasterxml.jackson.core.JsonProcessingException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.urbcomp.cupid.db.algorithm.kalman.AdaptiveKalmanFilter;
import org.urbcomp.cupid.db.algorithm.kalman.KalmanFilter;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class test {
    public static void main(String[] args) throws JsonProcessingException, AlgorithmExecuteException {

        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        TiHmmMapMatcher mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        Trajectory trajectory = ModelGenerator.generateTrajectory(50);
        MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
        List<GPSPoint> pointList = trajectory.getGPSPointList();
        List<GPSPoint> filterPointList = new ArrayList<>();
        double[][] F = new double[][]{
                {1, 0, 1, 0},
                {0, 1, 0, 1},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };
        double[][] H = new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };
        double pre_var = 15 * 15;
        double gps_var = 100 * 100;
        AdaptiveKalmanFilter kalmanFilter = new AdaptiveKalmanFilter(F, H, gps_var, pre_var);
        int index = 0;
        for (GPSPoint p : pointList){
            MapMatchedPoint mapMatchedPoint = mmTrajectory.getMmPtList().get(index);
            double[] estimate = kalmanFilter.process(p.getX(),p.getY(), mapMatchedPoint.getTime());
            GPSPoint filterPoint = new GPSPoint(p.getTime(), estimate[0], estimate[1]);
            filterPointList.add(filterPoint);
            kalmanFilter.update(mapMatchedPoint.getCandidatePoint().getX(), mapMatchedPoint.getCandidatePoint().getY(), mapMatchedPoint.getTime());
            kalmanFilter.updateNoiseCovariances(mapMatchedPoint.getCandidatePoint().getX(), mapMatchedPoint.getCandidatePoint().getY());
            index++;
        }
        Trajectory filterTrajectory = new Trajectory(trajectory.getTid(),trajectory.getOid(),filterPointList);
        System.out.println(trajectory.toGeoJSON());
        System.out.println(filterTrajectory.toGeoJSON());
    }

}
