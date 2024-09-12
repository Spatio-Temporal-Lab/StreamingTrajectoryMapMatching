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
import org.urbcomp.cupid.db.algorithm.history.generateHistoryProb;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;
import org.xml.sax.SAXException;

import javax.xml.bind.JAXBException;
import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import static org.junit.Assert.assertEquals;
import static org.urbcomp.cupid.db.model.sample.ModelGenerator.generateTrajectoryByStr;

public class StreamMapMatcherTest {

    private Trajectory trajectory;
    private TiHmmMapMatcher mapMatcher;
    private StreamMapMatcher mapMatcher2;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;
    private generateHistoryProb historyProb;

    @Before
    public void setUp() throws IOException {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
//        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
        historyProb = new generateHistoryProb();
    }

    @Test
    public void matchTrajToMapMatchedTraj() throws AlgorithmExecuteException, IOException, JAXBException, SAXException {
        MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory);
        System.out.println(trajectory.toGeoJSON());
        System.out.println(mmTrajectory.toGeoJSON());
        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }

    @Test
    public void matchTrajsToMapMatchedTrajs() throws AlgorithmExecuteException, IOException, JAXBException, SAXException {
        //TODO: fix bugs when dataID == 1003
        String trajFile = "data/output.txt";
        String outputFile = "map_matched_trajectories_stream.geojson";
        int success_count = 0;
        int trajectories_count = 1;
        try (
                InputStream in = ModelGenerator.class.getClassLoader().getResourceAsStream(trajFile);
                BufferedReader br = new BufferedReader(new InputStreamReader(Objects.requireNonNull(in)));
                BufferedWriter writer = new BufferedWriter(new FileWriter(outputFile))
        ) {
            String trajStr;
            int count = 0;
            while ((trajStr = br.readLine()) != null && count < trajectories_count) {
                count++;
                Trajectory trajectory = generateTrajectoryByStr(trajStr, 0);
                MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory);
                writer.write(mmTrajectory.toGeoJSON());
                writer.newLine();
                success_count++;
                System.out.println(success_count);
            }
        }
    }

    @Test
    public void accTest() throws AlgorithmExecuteException, IOException, JAXBException, SAXException {
        int testNum = 100;
        int startIndex = 1;
        testNum += startIndex;
        int sampleRate = 0;
        double countNum = 0;
        double accuracyNum = 0;
        for (; startIndex < testNum; startIndex++) {
            mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork), historyProb);
            System.out.println("Index: " + startIndex);
            trajectory = ModelGenerator.generateTrajectory(startIndex);
            Trajectory trajectorySampleRate = ModelGenerator.generateTrajectory(startIndex, sampleRate);
            //"-"+startIndex+
//            BufferedWriter writer = new BufferedWriter(new FileWriter("C:\\Users\\77595\\Desktop\\qgis\\ob"+".json"));
//            writer.write(trajectorySampleRate.toGeoJSON());
//            writer.close();
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
//            writer = new BufferedWriter(new FileWriter("C:\\Users\\77595\\Desktop\\qgis\\label"+".json"));
//            writer.write(mmTrajectory.toGeoJSON());
//            writer.close();
            MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(trajectorySampleRate);
            assert mmTrajectory2 != null;
            double accuracy = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory2, sampleRate);
            int length = mmTrajectory2.getMmPtList().size();
            countNum += length;
            double accNum = accuracy * length;
            accuracyNum += accNum;
            System.out.println("currAcc: " + accuracy);
            System.out.println("averageAcc: " + accuracyNum / countNum);
            System.out.println("accuracyNum: " + accuracyNum);
            System.out.println("countNum: " + countNum);
            System.out.println();
//            writer = new BufferedWriter(new FileWriter("C:\\Users\\77595\\Desktop\\qgis\\stream"+".json"));
//            writer.write(mmTrajectory2.toGeoJSON());
//            writer.close();
        }
    }
}
