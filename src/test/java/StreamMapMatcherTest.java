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
import it.unimi.dsi.fastutil.Hash;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AMM;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.Candidate;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;
import scala.Int;

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
    private AMM mapMatcher3;
    private ShortestPathPathRecover recover;

    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher3 = new AMM(roadNetwork);
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void matchTrajectoryToMapMatchedTrajectory() throws AlgorithmExecuteException, JsonProcessingException {
        MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0);
//        System.out.println(trajectory.toGeoJSON());
//        System.out.println(mmTrajectory.toGeoJSON());
//        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
//        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
//        System.out.println(pTrajectories.get(0).toGeoJSON());
//        assertEquals(1, pTrajectories.size());
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
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, sampleRate);
            Trajectory trajectory = ModelGenerator.generateTrajectory(index);

            // Perform map matching
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);

            // Uncomment to print the original and matched trajectories
            // printTrajectoryDetails(trajectory, mmTrajectory);

            // Compare accuracy for various beta values
            for (double beta = 0.0; beta <= 1.0; beta += 0.1) {
                MapMatchedTrajectory mmTrajectoryBeta = mapMatcher2.streamMapMatch(sampledTrajectory, beta);
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
        int testNum = 2000;
        int points;
        int totalPoints = 0;

        List<Double> weightList = new ArrayList<>();

        HashMap<Double, Integer> wrongPointsNum = new HashMap<>();
        HashMap<Double, Integer> totalWrongPointsNum = new HashMap<>();

        HashMap<Double, Long> executionTimeNum = new HashMap<>();
        HashMap<Double, Long> nonStreamExecutionTimeNum = new HashMap<>();

        addWeight(weightList);

        BufferedWriter accuracyWriter;

        for (int sampleRate = 0; sampleRate <= 20; sampleRate += 5) {
            String outputPath = "src/main/resources/data/accuracy/results/naive/sampleRate-" + sampleRate + ".csv";
            accuracyWriter = createWriter(outputPath);
            writeHeader(accuracyWriter);

            for (int startIndex = 1; startIndex < testNum; startIndex++) {
                System.out.println("Index: " + startIndex);

                // 生成轨迹
                trajectory = ModelGenerator.generateTrajectory(startIndex);
                // 采样生成轨迹
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(startIndex, sampleRate);
                // 记录轨迹点个数
                points = sampledTrajectory.getGPSPointList().size();
                // 记录总轨迹点个数
                totalPoints += points;
                // 计算离线匹配时间
                long nonStreamStartTime = System.nanoTime();
                MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);
                long nonStreamEndTime = System.nanoTime();
                long nonStreamExeTime = nonStreamEndTime - nonStreamStartTime;

                // Uncomment this line if you want to write the mmTrajectory to a file
                // writeTrajectoryToFile(mmTrajectory, "src/main/resources/data/match_result/offline/trajectory" + startIndex + ".json");

                // 处理不同 weight 情况
                processWeights(weightList, sampledTrajectory, mmTrajectory, totalWrongPointsNum, wrongPointsNum, executionTimeNum, nonStreamExecutionTimeNum, sampleRate, nonStreamExeTime);

                // Uncomment this line if you want to write the mmTrajectory2 to a file
                // writeTrajectoryToFile(mmTrajectory2, "src/main/resources/data/match_result/stream/modify/trajectory" + startIndex + ".json");

                // 写入结果
                writeResults(startIndex, weightList, accuracyWriter, wrongPointsNum, totalWrongPointsNum, executionTimeNum, nonStreamExecutionTimeNum, points, totalPoints);
            }

            totalPoints = 0;
            wrongPointsNum.clear();
            totalWrongPointsNum.clear();
            executionTimeNum.clear();
            nonStreamExecutionTimeNum.clear();
            accuracyWriter.close();
        }


    }

    private void addWeight(List<Double> weightList) {
        weightList.add(0.5);
//        for (int i = 0; i < 10; i++) {
//            weightList.add(0.0 + i * 0.1);
//        }
    }

    private BufferedWriter createWriter(String filePath) throws IOException {
        return new BufferedWriter(new FileWriter(filePath));
    }

    private void writeHeader(BufferedWriter writer) throws IOException {
        writer.write("index,weight,wrongPoints,points,totalWrongPoints,totalPoints,accuracy,accAccuracy,streamExeTime,nonStreamExeTime\n");
    }

    private void processWeights(List<Double> weightList, Trajectory sampledTrajectory, MapMatchedTrajectory mmTrajectory,
                                HashMap<Double, Integer> totalWrongPointsNum, HashMap<Double, Integer> wrongPointsNum,
                                HashMap<Double, Long> executionTimeNum, HashMap<Double, Long> nonStreamExecutionTimeNum,
                                int sampleRate, long nonStreamExeTime) throws AlgorithmExecuteException {
        double bestAcc = Double.MIN_VALUE;
        double bestWeight = -1;

        for (double weight : weightList) {
            // 统计流式匹配时间
            long startTime = System.nanoTime();
            MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(sampledTrajectory, weight);
            long endTime = System.nanoTime();
            long exeTime = endTime - startTime;

            assert mmTrajectory2 != null;

            // 获取匹配准确率和错误点个数
            EvaluateUtils.AccuracyResult accRes = EvaluateUtils.getAccuracyResult(mmTrajectory, mmTrajectory2, sampleRate);
            int wrongPoints = accRes.getWrongPoints();
            double accuracy = accRes.getAccuracy();

            // 保存当前错误点个数
            wrongPointsNum.put(weight, wrongPoints);

            // 保存总错误点个数
            totalWrongPointsNum.merge(weight, wrongPoints, Integer::sum);

            // 保存运行时间信息
            executionTimeNum.put(weight, exeTime);
            nonStreamExecutionTimeNum.put(weight, nonStreamExeTime);

            // 记录最佳准确率以及对应的权重
            if (accuracy > bestAcc) {
                bestAcc = accuracy;
                bestWeight = weight;
            }

            System.out.println("weight: " + weight + " accuracy: " + accuracy);

        }

        System.out.println("best weight: " + bestWeight + " accuracy: " + bestAcc);
    }

    private void writeResults(int index, List<Double> weightList, BufferedWriter accuracyWriter,
                              HashMap<Double, Integer> wrongPointsNum, HashMap<Double, Integer> totalWrongPointsNum,
                              HashMap<Double, Long> executionTimeNum, HashMap<Double, Long> nonStreamExecutionTimeNum,
                              int points, int totalPoints) throws IOException {
        // index, weight, wrong points, points, total wrong points, total points, accuracy, total accuracy, exeTime2, exeTime1
        for (double weight : weightList) {
            accuracyWriter.write(index + "," + weight + ","
                    + wrongPointsNum.get(weight) + "," + points + ","
                    + totalWrongPointsNum.get(weight) + "," + totalPoints + ","
                    + (1 - (wrongPointsNum.get(weight) * 1.0 / points)) + ","
                    + (1 - (totalWrongPointsNum.get(weight) * 1.0 / totalPoints)) + ","
                    + executionTimeNum.get(weight) / 1e6 + " ms,"
                    + nonStreamExecutionTimeNum.get(weight) / 1e6 + " ms\n");
        }
    }

    private void writeResults(int index, BufferedWriter accuracyWriter,
                              HashMap<Double, Integer> totalCountNum, HashMap<Double, Double> accuracyNum,
                              HashMap<Double, Integer> totalWrongPointsNum,
                              HashMap<Double, Long> executionTimeNum,
                              HashMap<Double, Long> nonStreamExecutionTimeNum) throws IOException {
        double weight = -1.0;
        accuracyWriter.write(index + "," + weight + "," +
                totalWrongPointsNum.get(weight) + "," + totalCountNum.get(weight) + "," +
                accuracyNum.get(weight) / totalCountNum.get(weight) + "," +
                (1 - (totalWrongPointsNum.get(weight) * 1.0 / totalCountNum.get(weight))) + "," +
                executionTimeNum.get(weight) / 1e6 + " ms," +
                nonStreamExecutionTimeNum.get(weight) / 1e6 + " ms\n");
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

    private void evaluateAMM(Trajectory trajectory, int id,
                             MapMatchedTrajectory mmTrajectory, int sampleRate,
                             HashMap<Double, Integer> totalCountNum, HashMap<Double, Integer> totalWrongPointsNum, HashMap<Double, Double> accuracyNum) {
        double v = mapMatcher3.mapMatch(trajectory, id);
        double weight = -1.0;
        List<Candidate> matchedList = mapMatcher3.getMatchedList();
        assert matchedList != null;

        MapMatchedTrajectory mmTrajectory3 = convertMatchedListToTrajectory(matchedList, trajectory.getTid(), trajectory.getOid());
        totalCountNum.merge(weight, matchedList.size(), Integer::sum);

        EvaluateUtils.AccuracyResult accRes = EvaluateUtils.getAccuracyResult(mmTrajectory, mmTrajectory3, sampleRate);
        totalWrongPointsNum.put(weight, accRes.getWrongPoints());
        double accNum = accRes.getAccuracy() * matchedList.size();
        accuracyNum.merge(weight, accNum, Double::sum);

        System.out.println("acc: " + accuracyNum.get(weight) / totalCountNum.get(weight));
        System.out.println("---------------------------------------------");
    }

    private MapMatchedTrajectory convertMatchedListToTrajectory(List<Candidate> matchedList, String tid, String oid) {
        List<MapMatchedPoint> mmPtList = new ArrayList<>();
        for (Candidate candidate : matchedList) {
            mmPtList.add(new MapMatchedPoint(candidate.parent.getObservation(), candidate.candidate));
        }
        return new MapMatchedTrajectory(tid, oid, mmPtList);
    }
}
