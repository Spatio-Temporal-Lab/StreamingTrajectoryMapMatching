import com.fasterxml.jackson.core.JsonProcessingException;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AMM;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.Candidate;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
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
        assert trajectory != null;
        double v = mapMatcher.mapMatch(trajectory, 0);
        System.out.println("matched length:" + v);
        List<Candidate> matchedList = mapMatcher.getMatchedList();
        assert matchedList != null;
        MapMatchedTrajectory mmTrajectory = convertMatchedListToTrajectory(matchedList, trajectory.getTid(), trajectory.getOid());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
    }

    @Test
    public void matchMultiTrajectories() throws IOException {
        List<Trajectory> trajectories = ModelGenerator.generateMultiTrajectory("data/output.txt", 1000, -1);
        for (int i = 0; i < trajectories.size(); i++) {
            Trajectory trajectory = trajectories.get(i);
            System.out.println("index " + (i + 1) + ":");
            System.out.println("trajectory size: " + trajectories.get(i).getGPSPointList().size());
            double v = mapMatcher.mapMatch(trajectories.get(i), i);
            System.out.println("matched length:" + v);
            MapMatchedTrajectory mmTrajectory = convertMatchedListToTrajectory(mapMatcher.getMatchedList(), trajectory.getTid(), trajectory.getOid());
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

    private MapMatchedTrajectory convertMatchedListToTrajectory(List<Candidate> matchedList, String tid, String oid) {
        List<MapMatchedPoint> mmPtList = new ArrayList<>();
        for (Candidate candidate : matchedList) {
            mmPtList.add(new MapMatchedPoint(candidate.parent.getObservation(), candidate.candidate));
        }
        return new MapMatchedTrajectory(tid, oid, mmPtList);
    }
}