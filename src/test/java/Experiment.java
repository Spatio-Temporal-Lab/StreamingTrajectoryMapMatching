import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.Candidate;
import org.urbcomp.cupid.db.algorithm.mapmatch.aomm.AommMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.dwrmm.DwrmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.DynamicWeightAdjuster;
import org.urbcomp.cupid.db.algorithm.weightAdjuster.FixedWeightAdjuster;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class Experiment {
    private Trajectory trajectory;
    private TiHmmMapMatcher labelMapMatcher;
    private StreamMapMatcher ourMapMatcher;
    private StreamMapMatcher baseMapMatcher;
    private AmmMapMatcher ammMapMatcher;
    private AommMapMatcher aommMapMatcher;
    private DwrmmMapMatcher dwrmmMapMatcher;

    public static void main(String[] args) {
        Experiment experiment = new Experiment();
        experiment.accuracyAndEfficiencyTest();
    }

    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        labelMapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        ourMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        baseMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        ammMapMatcher = new AmmMapMatcher(roadNetwork);
        aommMapMatcher = new AommMapMatcher(roadNetwork);
        dwrmmMapMatcher = new DwrmmMapMatcher(roadNetwork);
    }

    @Test
    public void accuracyAndEfficiencyTest() {
        PrintStream originalOut = System.out; // 保存默认的输出流
        PrintStream errorLogStream = null;

        try {
            // 将输出重定向到日志文件
            File resultFile = new File("result.txt");
            File indexFile = new File("index.txt");
            File errorFile = new File("error.txt");
            PrintStream resultLogStream = new PrintStream(Files.newOutputStream(resultFile.toPath()));
            PrintStream indexLogStream = new PrintStream(Files.newOutputStream(indexFile.toPath()));
            errorLogStream = new PrintStream(Files.newOutputStream(errorFile.toPath()));

            setUp();
            long totalDelay = 0; // 总延迟，单位为纳秒
            double averageDelay;
            int startIndex = 1;
            int testNum = 2000;
            int windowSize = 20;
            boolean OURS = true;
            boolean BASE = false;
            boolean AMM = false;
            boolean AOMM = false;
            boolean DWRMM = false;
            int[] sampleRates = {6};
            testNum += startIndex;
            int originalSampleRate = 3;
            for (int resultSampleRate : sampleRates) {

                // our method
                if (OURS) {
                    resultLogStream.println("---- OURS ----");
                    indexLogStream.println("---- OURS ----");
                    for (int index = startIndex; index < testNum; index++) {
                        System.out.println("index: " + index);
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSampleRate);
                        DynamicWeightAdjuster dynamicWeightAdjuster = new DynamicWeightAdjuster();
//                            FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                        // 输出当前轨迹的索引到 indexLogStream
                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        // 计算准确率和延迟
                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = ourMapMatcher.onlineStreamMapMatch(sampledTrajectory, dynamicWeightAdjuster, windowSize);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSampleRate);
                        resultLogStream.println("average backtrack time: " + ourMapMatcher.getDelayTime() / ourMapMatcher.getDelayNums());
                    }
                    // 准确率
                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());

                    // 平均延迟
                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    // 平均回溯延迟
                    resultLogStream.println("backtrack num: " + ourMapMatcher.getDelayNums());
                    resultLogStream.println("backtrack time: " + ourMapMatcher.getDelayTime());
                    resultLogStream.println("average backtrack time: " + ourMapMatcher.getDelayTime() / ourMapMatcher.getDelayNums());

                    EvaluateUtils.reset();
                    totalDelay = 0;

                }

                // base onlineHmm
                if (BASE) {
                    resultLogStream.println("---- Base onlineHmm ---- sampleRate: " + resultSampleRate);
                    indexLogStream.println("---- Base onlineHmm ---- sampleRate: " + resultSampleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSampleRate);
                        FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                        // 输出当前轨迹的索引到 indexLogStream
                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        // 计算准确率和延迟
                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = baseMapMatcher.streamMapMatch(sampledTrajectory, fixedWeightAdjuster);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSampleRate);
                    }
                    // 准确率
                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());

                    // 平均延迟
                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    EvaluateUtils.reset();
                    totalDelay = 0;
                }


                // AMM
                if (AMM) {
                    resultLogStream.println("---- AMM ---- sampleRate: " + resultSampleRate);
                    indexLogStream.println("---- AMM ---- sampleRate: " + resultSampleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSampleRate);

                        // 输出当前轨迹的索引到 indexLogStream
                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        // 计算准确率和延迟
                        long startTime = System.nanoTime();
                        ammMapMatcher.mapMatch(sampledTrajectory, index);
                        if (ammMapMatcher.getMatchedList() == null) {
                            continue;
                        }
                        MapMatchedTrajectory result = convertMatchedListToTrajectory(ammMapMatcher.getMatchedList(), trajectory.getTid(), trajectory.getOid());
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSampleRate);
                    }
                    // 准确率
                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());

                    // 平均延迟
                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    EvaluateUtils.reset();
                    totalDelay = 0;
                }


                // AOMM
                if (AOMM) {
                    resultLogStream.println("---- AOMM ---- sampleRate: " + resultSampleRate);
                    indexLogStream.println("---- AOMM ---- sampleRate: " + resultSampleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSampleRate);

                        // 输出当前轨迹的索引到 indexLogStream
                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        // 计算准确率和延迟
                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = aommMapMatcher.aommMapMatch(sampledTrajectory);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSampleRate);
                    }
                    // 准确率
                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());
                    resultLogStream.println("pointNums: " + EvaluateUtils.getTotalNum());

                    // 平均延迟
                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    EvaluateUtils.reset();
                    totalDelay = 0;
                }


                // DW-RMM
                if (DWRMM) {
                    resultLogStream.println("---- DW-RMM ---- sampleRate: " + resultSampleRate);
                    indexLogStream.println("---- DW-RMM ---- sampleRate: " + resultSampleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSampleRate);

                        // 输出当前轨迹的索引到 indexLogStream
                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        // 计算准确率和延迟
                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = dwrmmMapMatcher.dwrmmMapMatch(sampledTrajectory);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSampleRate);
                    }
                    // 准确率
                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());

                    // 平均延迟
                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            errorLogStream.println("Exception: " + e);
            for (StackTraceElement element : e.getStackTrace()) {
                errorLogStream.println("\tat " + element);
            }
        } finally {
            // 恢复默认的输出流
            if (errorLogStream != null) {
                errorLogStream.close();
            }
            System.setOut(originalOut);
        }
    }

    private MapMatchedTrajectory convertMatchedListToTrajectory(List<Candidate> matchedList, String tid, String oid) {
        List<MapMatchedPoint> mmPtList = new ArrayList<>();
        for (Candidate candidate : matchedList) {
            mmPtList.add(new MapMatchedPoint(candidate.parent.getObservation(), candidate.candidate));
        }
        return new MapMatchedTrajectory(tid, oid, mmPtList);
    }

    @Test
    public void saveMatchResultTest() {
        int originalSampleRate = 3;
        int resultSampleRate = 3;
        int testNum = 1000;
        int windowSize = -1;
        String outputBasePath = "D:\\Results\\MapMatching\\match\\CD-Taxis";

        for (int i = 1; i < testNum; i++) {
            System.out.println("trajectory: " + i);
            try {
                // Generate the trajectory
                Trajectory trajectory = ModelGenerator.generateTrajectory(i);
                // Generate sampled trajectory
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, originalSampleRate, resultSampleRate);

                // Offline HMM label
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);
                // Perform matches with different models
                MapMatchedTrajectory ourResult = ourMapMatcher.onlineStreamMapMatch(sampledTrajectory, new DynamicWeightAdjuster(), windowSize);
                MapMatchedTrajectory baseResult = baseMapMatcher.streamMapMatch(sampledTrajectory, new FixedWeightAdjuster());
                ammMapMatcher.mapMatch(sampledTrajectory, i);
                MapMatchedTrajectory ammResult = convertMatchedListToTrajectory(ammMapMatcher.getMatchedList(), trajectory.getTid(), trajectory.getOid());
                MapMatchedTrajectory aommResult = aommMapMatcher.aommMapMatch(sampledTrajectory);
                MapMatchedTrajectory dwrmmResult = dwrmmMapMatcher.dwrmmMapMatch(sampledTrajectory);

                // Save results to respective folders
                saveMatchResult(ammResult, outputBasePath + "\\AMM\\result_" + i + ".txt");
                saveMatchResult(aommResult, outputBasePath + "\\AOMM\\result_" + i + ".txt");
                saveMatchResult(ourResult, outputBasePath + "\\ERA-MM\\result_" + i + ".txt");
                saveMatchResult(dwrmmResult, outputBasePath + "\\DW-RMM\\result_" + i + ".txt");
                saveMatchResult(labelResult, outputBasePath + "\\OHMM\\result_" + i + ".txt");
                saveMatchResult(baseResult, outputBasePath + "\\BASE\\result_" + i + ".txt");

            } catch (AlgorithmExecuteException e) {
                e.printStackTrace();
            }
        }
    }

    private void saveMatchResult(MapMatchedTrajectory result, String filePath) {
        try {
            // Create the parent directories if they don't exist
            Files.createDirectories(Paths.get(filePath).getParent());

            // Write the result to the file
            try (PrintStream resultLogStream = new PrintStream(Files.newOutputStream(Paths.get(filePath)))) {
                resultLogStream.println(result.toGeoJSON()); // Adjust this line as needed to format the output
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
