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

import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.history.generateHistoryProb;
import org.urbcomp.cupid.db.algorithm.mapmatch.aomm.AommMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
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
import java.util.List;
import java.util.Objects;

import static org.junit.Assert.assertEquals;
import static org.urbcomp.cupid.db.model.sample.ModelGenerator.generateTrajectoryByStr;

public class StreamMapMatcherTest {

    private Trajectory trajectory;
    private TiHmmMapMatcher mapMatcher;
    private StreamMapMatcher mapMatcher2;
    private AommMapMatcher aommMapMatcher;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    @Before
    public void setUp() throws IOException {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
//        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
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
//        String trajectoryWritePath = "C:\\Users\\77595\\Desktop\\qgis\\ob"+".json";;
//        String trajectoryWritePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\ob"+".json";
//        String labelMapMatcherWritePath = "C:\\Users\\77595\\Desktop\\qgis\\label"+".json";
//        String labelMapMatcherWritePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\label"+".json";
//        String streamMapMatcherWritePath = "C:\\Users\\77595\\Desktop\\qgis\\stream"+".json";
//        String streamMapMatcherWritePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\stream"+".json";

        int testNum = 10;
        int startIndex = 1;
        testNum += startIndex;
        int sampleRate = 0;
        for (; startIndex < testNum; startIndex++) {
            System.out.println("Index: " + startIndex);
            trajectory = ModelGenerator.generateTrajectory(startIndex);

            // offline hmm(label)
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(startIndex, sampleRate);
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
//            BufferedWriter writer = new BufferedWriter(new FileWriter(trajectoryWritePath));
//            writer.write(sampledTrajectory.toGeoJSON());
//            writer.close();
//            writer = new BufferedWriter(new FileWriter(labelMapMatcherWritePath));
//            writer.write(mmTrajectory.toGeoJSON());
//            writer.close();

            // our method
            mapMatcher2 = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
            MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(sampledTrajectory);
            assert mmTrajectory2 != null;
            EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory2, sampleRate);

            // aomm
//            aommMapMatcher = new AommMapMatcher(roadNetwork);
//            MapMatchedTrajectory mmTrajectory3 = aommMapMatcher.AommMapMatch(sampledTrajectory);
//            assert mmTrajectory3 != null;
//            EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory3, sampleRate);

            System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
            System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
            System.out.println("pointNum: " + EvaluateUtils.getTotalNum());
            System.out.println();

//            writer = new BufferedWriter(new FileWriter(streamMapMatcherWritePath));
//            writer.write(mmTrajectory3.toGeoJSON());
//            writer.close();
        }
    }
}
