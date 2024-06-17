/* 
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

import com.fasterxml.jackson.core.JsonProcessingException;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.predict.LSTMModel;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.*;
import java.util.List;
import java.util.Objects;

import static org.junit.Assert.assertEquals;
import static org.urbcomp.cupid.db.model.sample.ModelGenerator.generateTrajectoryByStr;

public class StreamMapMatcherTest {

    private Trajectory trajectory;
    private TiHmmMapMatcher mapMatcher;
    private StreamMapMatcher mapMatcher2;
    private ShortestPathPathRecover recover;

    @Before
    public void setUp() throws IOException {
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void matchTrajToMapMatchedTraj() throws AlgorithmExecuteException, JsonProcessingException {
        MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory,1,0);
        System.out.println(trajectory.toGeoJSON());
        System.out.println(mmTrajectory.toGeoJSON());
        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }

    @Test
    public void matchTrajCompare() throws AlgorithmExecuteException, JsonProcessingException {
        trajectory = ModelGenerator.generateTrajectory(50);
        System.out.println(trajectory.toGeoJSON());
        MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
        System.out.println(mmTrajectory.toGeoJSON());
        MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(trajectory,1,0);
        System.out.println(mmTrajectory2.toGeoJSON());
        double acc1 = EvaluateUtils.getAccuracy(mmTrajectory,mmTrajectory2);
        System.out.println("alpha:1 beta:0 --- ACC:" + acc1);
        MapMatchedTrajectory mmTrajectory3 = mapMatcher2.streamMapMatch(trajectory,0.5,0.5);
        System.out.println(mmTrajectory3.toGeoJSON());
        double acc2 = EvaluateUtils.getAccuracy(mmTrajectory,mmTrajectory3);
        System.out.println("alpha:0.5 beta:0.5 --- ACC:" + acc2);
        MapMatchedTrajectory mmTrajectory4 = mapMatcher2.streamMapMatch(trajectory,0,1);
        System.out.println(mmTrajectory4.toGeoJSON());
        double acc3 = EvaluateUtils.getAccuracy(mmTrajectory,mmTrajectory4);
        System.out.println("alpha:0 beta:1 --- ACC:" + acc3);
    }

    @Test
    public void matchTrajsToMapMatchedTrajs() throws AlgorithmExecuteException, IOException {
        String trajFile = "data/output.txt";
        String outputFile = "map_matched_trajectories_stream.geojson";
        int success_count = 0;
        int trajectories_count = 2000;
        try (
                InputStream in = ModelGenerator.class.getClassLoader().getResourceAsStream(trajFile);
                BufferedReader br = new BufferedReader(new InputStreamReader(Objects.requireNonNull(in)));
                BufferedWriter writer = new BufferedWriter(new FileWriter(outputFile))
        ) {
            String trajStr;
            int count = 0;
            while ((trajStr = br.readLine()) != null && count < trajectories_count) {
                count ++;
                Trajectory trajectory = generateTrajectoryByStr(trajStr, -1);
                MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory,0.5,0.5);
                writer.write(mmTrajectory.toGeoJSON());
                writer.newLine();
                success_count++;
                System.out.println(success_count);
            }
        }
    }
}
