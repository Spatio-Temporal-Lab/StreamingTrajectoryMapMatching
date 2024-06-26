import com.fasterxml.jackson.core.JsonProcessingException;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AMM;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class AdaptiveMapMatcherTest {
    private AMM mapMatcher;
    private ShortestPathPathRecover recover;

    @Before
    public void setUp() {
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new AMM(roadNetwork);
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void matchSingleTrajectory() throws JsonProcessingException {
        Trajectory trajectory = ModelGenerator.generateSingleTrajectory("data/output.txt", 0, -1);
        double v = 0;
        if (trajectory != null) {
            v = mapMatcher.mapMatch(trajectory, 0);
        }
        System.out.println("matched length:" + v);
        MapMatchedTrajectory mmTrajectory = mapMatcher.getMatchedTraj();
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
    }

    @Test
    public void matchMultiTrajectories() throws IOException {
        List<Trajectory> trajectories = ModelGenerator.generateMultiTrajectory("data/output.txt", 1000, -1);
        for (int i = 0; i < trajectories.size(); i++) {
            System.out.println("index " + (i + 1) + ":");
            System.out.println("trajectory size: " + trajectories.get(i).getGPSPointList().size());
            double v = mapMatcher.mapMatch(trajectories.get(i), i);
            System.out.println("matched length:" + v);
            MapMatchedTrajectory mmTrajectory = mapMatcher.getMatchedTraj();
            System.out.println("matched size:" + mmTrajectory.getMmPtList().size());
            writeFile(mmTrajectory);
        }
    }

    @Test
    public void testAccuracy() {
        String labelPath = ".\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = ".\\src\\main\\resources\\data\\map_matched_trajectories_amm.geojson";
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath, 1000);
        System.out.println("Accuracy:" + accuracy);
    }

    private void writeFile(MapMatchedTrajectory mmTrajectory) throws IOException {
        try (
                BufferedWriter writer = new BufferedWriter(new FileWriter("map_matched_trajectories_amm.geojson", true))
        ) {
            writer.write(mmTrajectory.toGeoJSON());
            writer.newLine();
            System.out.println("----------------success write!----------------");
        }
    }
}
