import com.fasterxml.jackson.core.JsonProcessingException;
import org.urbcomp.cupid.db.algorithm.kalman.GPSPositionSpeedFilter;
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

public class test2 {
    public static void main(String[] args) throws JsonProcessingException, AlgorithmExecuteException {

        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        TiHmmMapMatcher mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        Trajectory trajectory = ModelGenerator.generateTrajectory(6);
        MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
        List<GPSPoint> pointList = trajectory.getGPSPointList();
        List<GPSPoint> filterPointList = new ArrayList<>();

        GPSPositionSpeedFilter filter = new GPSPositionSpeedFilter(100., 100.);

        int index = 1;
        GPSPoint filterPoint = new GPSPoint(pointList.get(0).getTime(), pointList.get(0).getLng(), pointList.get(0).getLat());
        filterPointList.add(filterPoint);
        for (;index<pointList.size();index++){
            MapMatchedPoint mapMatchedPoint = mmTrajectory.getMmPtList().get(index);
            filter.updatePosition(pointList.get(index).getLng(), pointList.get(index).getLat(), pointList.get(index).getTime().getTime() - pointList.get(index-1).getTime().getTime());

            double[] position = filter.getPosition();

            filterPoint = new GPSPoint(pointList.get(index).getTime(), position[0], position[1]);
            filterPointList.add(filterPoint);
        }
        Trajectory filterTrajectory = new Trajectory(trajectory.getTid(),trajectory.getOid(),filterPointList);
//        System.out.println(trajectory.toGeoJSON());
//        System.out.println(mmTrajectory.toGeoJSON());
        System.out.println(filterTrajectory.toGeoJSON());
    }
}
