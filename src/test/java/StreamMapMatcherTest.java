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
    public void setUp() throws IOException {
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        mapMatcher2 = new StreamMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void matchTrajToMapMatchedTraj() throws AlgorithmExecuteException, JsonProcessingException {
        MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0, false);
        System.out.println(trajectory.toGeoJSON());
        System.out.println(mmTrajectory.toGeoJSON());
        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }

    @Test
    public void matchTrajCompare() throws AlgorithmExecuteException, JsonProcessingException {
        boolean useRectify = false;
        int i = 1000;
        int sampleRate = 0;
        int size = 0;
        double normal;
        double fix;
        double minValue = Double.MAX_VALUE;
        double totalFix = 0;
        double totalSize = 0;
        for (int index = 1; index < i; index++) {
            System.out.println("Index:" + index);
            Trajectory filterTrajectory;
            Trajectory trajectorySampleRate = ModelGenerator.generateTrajectory(index, sampleRate);
            trajectory = ModelGenerator.generateTrajectory(index);
//            System.out.println("--------------------------------");
////            System.out.println(trajectory.toGeoJSON());
//            System.out.println("--------------------------------");
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);
//            System.out.println(mmTrajectory.toGeoJSON());
//            System.out.println("--------------------------------");
//            MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(trajectorySampleRate, 1, 0.5);
////            System.out.println(mmTrajectory2.toGeoJSON());
//            filterTrajectory = new Trajectory(trajectory.getTid(), trajectory.getOid(), mapMatcher2.filteredPoints);
////            System.out.println(filterTrajectory.toGeoJSON());
//            double acc1 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory2, sampleRate);
//            System.out.println("alpha:1 beta:0 --- ACC:" + acc1 + "totalPoints: " + mmTrajectory2.getMmPtList().size());
//            System.out.println("--------------------------------");
//            MapMatchedTrajectory mmTrajectory3 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0, 0.7);
////            System.out.println(mmTrajectory3.toGeoJSON());
//            filterTrajectory = new Trajectory(trajectory.getTid(), trajectory.getOid(), mapMatcher2.filteredPoints);
////            System.out.println(filterTrajectory.toGeoJSON());
//            double acc2 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory3, sampleRate);
//            System.out.println("alpha:0.5 beta:0.5 --- ACC:" + acc2 + "totalPoints: " + mmTrajectory3.getMmPtList().size());
//            System.out.println("--------------------------------");

            MapMatchedTrajectory mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.5, useRectify);
            double acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            normal = (1 - acc3) * size;
            size = mmTrajectory4.getMmPtList().size();
            totalSize += size;
            System.out.println("totalPoint: " + "totalPoints: " + mmTrajectory4.getMmPtList().size());
            System.out.println("beta:0.5 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.0, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.0 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.1, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.1 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.2, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.2 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.3, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.3 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.4, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.4 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.6, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.6 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.7, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.7 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.8, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.8 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 0.9, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:0.9 --- ACC:" + acc3);
            mmTrajectory4 = mapMatcher2.streamMapMatch(trajectorySampleRate, 1.0, useRectify);
            acc3 = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory4, sampleRate);
            fix = (1 - acc3) * size;
            minValue = Math.min(fix, minValue);
            System.out.println("beta:1.0 --- ACC:" + acc3);

            if (normal > minValue) {
                totalFix += normal - minValue;
            } else {
                totalFix += 0;
            }
            System.out.println("fix prob: " + totalFix / totalSize);
        }


    }

    @Test
    public void matchTrajsToMapMatchedTrajs() throws AlgorithmExecuteException, IOException {
        //TODO: fix bugs when dataID == 1003
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
                MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0.5, false);
                writer.write(mmTrajectory.toGeoJSON());
                writer.newLine();
                success_count++;
                System.out.println(success_count);
            }
        }
    }

    @Test
    public void weightTest() throws AlgorithmExecuteException, JsonProcessingException, IOException {
        int testNum = 100;  // 要测试的总次数
        int startIndex = 1; // 测试的起始索引
        testNum += startIndex;
        int sampleRate = 9; // 采样率
        List<Double> weightList = new ArrayList<>(); // 权重列表
        HashMap<Double, Integer> countNum = new HashMap<>(); // 权重对应的点计数
        HashMap<Double, Double> accuracyNum = new HashMap<>(); // 权重对应的准确率
        addWeight(weightList); // 添加权重值到权重列表

        // 创建用于写入准确率结果的文件
        BufferedWriter accuracyWriter = new BufferedWriter(
                new FileWriter("src/main/resources/data/accuracy/stream/sample_rate/9/no-rectify.csv"));
        accuracyWriter.write("index," + "weight," + "accuracy" + "\n");

        for (; startIndex < testNum; startIndex++) {
            System.out.println("Index: " + startIndex);

            // 生成轨迹和采样轨迹
            trajectory = ModelGenerator.generateTrajectory(startIndex);
            Trajectory trajectorySampleRate = ModelGenerator.generateTrajectory(startIndex, sampleRate);

            // 使用mapMatcher进行轨迹匹配
            MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);

            // 写入mapMatcher的匹配结果到文件
//            try (BufferedWriter writer = new BufferedWriter(
//                    new FileWriter("src/main/resources/data/match_result/offline/trajectory" + startIndex + ".json"))) {
//                writer.write(mmTrajectory.toGeoJSON());
//            }

            MapMatchedTrajectory mmTrajectory2 = null; // 初始化第二个匹配结果
            double bestAcc = Double.MIN_VALUE; // 初始化最佳准确率
            int length = 0; // 初始化匹配点长度

            // 遍历权重列表，进行匹配并计算准确率
            for (double weight : weightList) {
                mmTrajectory2 = mapMatcher2.streamMapMatch(trajectorySampleRate, weight, false);
                assert mmTrajectory2 != null;
                countNum.merge(weight, mmTrajectory2.getMmPtList().size(), Integer::sum);
                double accuracy = EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory2, sampleRate);
                bestAcc = Math.max(bestAcc, accuracy);
                length = mmTrajectory2.getMmPtList().size();
                double accNum = accuracy * length;
                accuracyNum.merge(weight, accNum, Double::sum);
                System.out.println("weight: " + weight + " acc: " + accuracyNum.get(weight) / countNum.get(weight));
            }

            // 写入mapMatcher2的匹配结果到文件
//            try (BufferedWriter writer = new BufferedWriter(
//                    new FileWriter("src/main/resources/data/match_result/stream/rectify/trajectory" + startIndex + ".json"))) {
//                if (mmTrajectory2 != null) {
//                    writer.write(mmTrajectory2.toGeoJSON());
//                }
//            }

            // 计算并合并最佳准确率
            double bestAccNum = bestAcc * length;
            accuracyNum.merge(2.0, bestAccNum, Double::sum);
            System.out.println("best acc: " + accuracyNum.get(2.0) / countNum.get(0.5));
            System.out.println("---------------------------------------------");

            // 写入每次循环后的准确率到文件中
            for (double weight : weightList) {
                accuracyWriter.write(startIndex + "," + weight + "," + accuracyNum.get(weight) / countNum.get(weight) + "\n");
            }
        }

        accuracyWriter.close(); // 关闭准确率结果文件
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

    @Test
    public void generateMapMatchResults() throws IOException, AlgorithmExecuteException {
        String trajFile = "data/prepare/trajectories_chengdu.txt";
        String outputFile = "src/main/resources/data/no_rectify_trajectory.geojson";
        int success_count = 0;
        int trajectories_count = 50;
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
                System.out.println("traj size: " + trajectory.getGPSPointList().size());
                MapMatchedTrajectory mmTrajectory = mapMatcher2.streamMapMatch(trajectory, 0.5, false);
                System.out.println("traj size: " + mmTrajectory.getMmPtList().size());
                writer.write(mmTrajectory.toGeoJSON());
                writer.newLine();
                success_count++;
                System.out.println(success_count);
            }
        }
    }

}
