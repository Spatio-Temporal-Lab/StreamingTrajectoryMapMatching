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

    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void matchTrajectoryToMapMatchedTrajectory() throws AlgorithmExecuteException, JsonProcessingException {
        MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0);
//        System.out.println(trajectory.toGeoJSON());
//        System.out.println(mmTrajectory.toGeoJSON());
        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
//        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }

    @Test
    public void matchTrajectoryCompare() throws AlgorithmExecuteException, JsonProcessingException {
        int totalIterations = 1000;
        int sampleRate = 0;
        int size = 0;
        double normal;
        double fix;
        double minValue = Double.MAX_VALUE;
        double totalFix = 0;
        double totalSize = 0;

        for (int index = 1; index < totalIterations; index++) {
            System.out.println("Index:" + index);

            // Generate trajectories
            Trajectory trajectorySampleRate = ModelGenerator.generateTrajectory(index, sampleRate);
            Trajectory trajectory = ModelGenerator.generateTrajectory(index);

            // Perform map matching
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);

            // Uncomment to print the original and matched trajectories
            // printTrajectoryDetails(trajectory, mmTrajectory);

            // Compare accuracy for various beta values
            for (double beta = 0.0; beta <= 1.0; beta += 0.1) {
                MapMatchedTrajectory mmTrajectoryBeta = mapMatcher2.streamMapMatch(trajectorySampleRate, beta);
                double accuracy = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectoryBeta, sampleRate);
                size = mmTrajectoryBeta.getMmPtList().size();
                totalSize += size;

                // Calculate and print accuracy for current beta
                fix = (1 - accuracy) * size;
                minValue = Math.min(fix, minValue);
                System.out.println("beta:" + beta + " --- ACC:" + accuracy);
            }

            // Calculate normal accuracy
            normal = (1 - EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory, sampleRate)) * size;

            // Adjust total fix value
            totalFix += (normal > minValue) ? normal - minValue : 0;
            System.out.println("Fix prob: " + totalFix / totalSize);
        }
    }

    @Test
    public void matchTrajectoriesToMapMatchedTrajectories() throws AlgorithmExecuteException, IOException {
        String trajFile = "data/trajectories_chengdu.txt";
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
                MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0.5);
                writer.write(mmTrajectory.toGeoJSON());
                writer.newLine();
                success_count++;
                System.out.println(success_count);
            }
        }
    }

    @Test
    public void weightTest() throws AlgorithmExecuteException, IOException {
        int testNum = 15;
        int startIndex = 1;
        testNum += startIndex;
        int sampleRate = 0;
        List<Double> weightList = new ArrayList<>();
        HashMap<Double, Integer> countNum = new HashMap<>();
        HashMap<Double, Double> accuracyNum = new HashMap<>();
        HashMap<Double, Integer> wrongPointsNum = new HashMap<>();

        addWeight(weightList);

        BufferedWriter accuracyWriter = createWriter("src/main/resources/data/accuracy/ablation/transition/sampleRate-0.csv");
        writeHeader(accuracyWriter);

        for (; startIndex < testNum; startIndex++) {
            System.out.println("Index: " + startIndex);
            Trajectory trajectory = ModelGenerator.generateTrajectory(startIndex);
            Trajectory trajectorySampleRate = ModelGenerator.generateTrajectory(startIndex, sampleRate);
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);

            // Uncomment this line if you want to write the mmTrajectory to a file
            // writeTrajectoryToFile(mmTrajectory, "src/main/resources/data/match_result/offline/trajectory" + startIndex + ".json");

            processWeights(weightList, trajectorySampleRate, mmTrajectory, countNum, accuracyNum, wrongPointsNum, sampleRate);

            // Uncomment this line if you want to write the mmTrajectory2 to a file
            // writeTrajectoryToFile(mmTrajectory2, "src/main/resources/data/match_result/stream/modify/trajectory" + startIndex + ".json");

            writeResults(startIndex, weightList, accuracyWriter, countNum, accuracyNum, wrongPointsNum);
            wrongPointsNum.clear();
        }

        accuracyWriter.close();
    }

    private void addWeight(List<Double> weightList) {
        weightList.add(0.5);
//        weightList.add(0.0);
//        weightList.add(0.1);
//        weightList.add(0.2);
//        weightList.add(0.3);
//        weightList.add(0.4);
//        weightList.add(0.6);
//        weightList.add(0.7);
//        weightList.add(0.8);
//        weightList.add(0.9);
//        weightList.add(1.0);
    }

    private BufferedWriter createWriter(String filePath) throws IOException {
        return new BufferedWriter(new FileWriter(filePath));
    }

    private void writeHeader(BufferedWriter writer) throws IOException {
        writer.write("index,weight,wrongPoints,totalPoints,accAccuracy,accuracy\n");
    }

    private void processWeights(List<Double> weightList, Trajectory trajectorySampleRate, MapMatchedTrajectory mmTrajectory,
                                HashMap<Double, Integer> countNum, HashMap<Double, Double> accuracyNum,
                                HashMap<Double, Integer> wrongPointsNum, int sampleRate) throws AlgorithmExecuteException {
        double bestAcc = Double.MIN_VALUE;
        int length = 0;

        for (double weight : weightList) {
            MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(trajectorySampleRate, weight);
            assert mmTrajectory2 != null;
            countNum.merge(weight, mmTrajectory2.getMmPtList().size(), Integer::sum);

            EvaluateUtils.AccuracyResult accRes = EvaluateUtils.getAccuracyResult(mmTrajectory, mmTrajectory2, sampleRate);
            wrongPointsNum.put(weight, accRes.getWrongPoints());

            bestAcc = Math.max(bestAcc, accRes.getAccuracy());
            length = mmTrajectory2.getMmPtList().size();

            double accNum = accRes.getAccuracy() * length;
            accuracyNum.merge(weight, accNum, Double::sum);
            System.out.println("weight: " + weight + " acc: " + accuracyNum.get(weight) / countNum.get(weight));
        }

        double bestAccNum = bestAcc * length;
        accuracyNum.merge(2.0, bestAccNum, Double::sum);
        System.out.println("best acc: " + accuracyNum.get(2.0) / countNum.get(0.5));
        System.out.println("---------------------------------------------");
    }

    private void writeResults(int index, List<Double> weightList, BufferedWriter accuracyWriter,
                              HashMap<Double, Integer> countNum, HashMap<Double, Double> accuracyNum,
                              HashMap<Double, Integer> wrongPointsNum) throws IOException {
        for (double weight : weightList) {
            accuracyWriter.write(index + "," + weight + "," +
                    wrongPointsNum.get(weight) + "," + countNum.get(weight) + "," +
                    accuracyNum.get(weight) / countNum.get(weight) + "," +
                    (1 - (wrongPointsNum.get(weight) * 1.0 / countNum.get(weight))) + "\n");
        }
    }

    private void writeTrajectoryToFile(MapMatchedTrajectory trajectory, String filePath) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
            if (trajectory != null) {
                writer.write(trajectory.toGeoJSON());
            }
        }
    }

    private void printTrajectoryDetails(Trajectory trajectory, MapMatchedTrajectory mmTrajectory) throws JsonProcessingException {
        System.out.println("--------------------------------");
        System.out.println("Original Trajectory:");
        System.out.println(trajectory.toGeoJSON());
        System.out.println("Matched Trajectory:");
        System.out.println(mmTrajectory.toGeoJSON());
        System.out.println("--------------------------------");
    }
}
