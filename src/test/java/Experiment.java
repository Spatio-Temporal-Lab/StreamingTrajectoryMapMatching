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
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
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

    public void setUp(){
        trajectory = ModelGenerator.generateTrajectory();
        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
        labelMapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        ourMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        baseMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        ammMapMatcher = new AmmMapMatcher(roadNetwork);
        aommMapMatcher = new AommMapMatcher(roadNetwork);
        dwrmmMapMatcher = new DwrmmMapMatcher(roadNetwork);
    }

    @Test
    public void accuracyAndEfficiencyTest() {
        PrintStream originalOut = System.out;
        PrintStream errorLogStream = null;

        try {

            File resultFile = new File("result.txt");
            File indexFile = new File("index.txt");
            File errorFile = new File("error.txt");
            PrintStream resultLogStream = new PrintStream(new FileOutputStream(resultFile));
            PrintStream indexLogStream = new PrintStream(new FileOutputStream(indexFile));
            errorLogStream = new PrintStream(new FileOutputStream(errorFile));

            setUp();
            long totalDelay = 0;
            double averageDelay;
            int startIndex = 1;
            int testNum = 2000;
            int windowSize = 20;
            boolean OURS = true;
            boolean BASE = false;
            boolean AMM = false;
            boolean AOMM = false;
            boolean DWRMM = false;
            int[] samepleRates = {6};
            testNum += startIndex;
            int originalSampleRate = 3;
            for (int resultSamepleRate: samepleRates) {

                // our method
                if (OURS) {
                    resultLogStream.println("---- OURS ---- sampleRate: " + resultSamepleRate);
                    indexLogStream.println("---- OURS ---- sampleRate: " + resultSamepleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSamepleRate);
                        DynamicWeightAdjuster dynamicWeightAdjuster = new DynamicWeightAdjuster();
//                      FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = ourMapMatcher.onlineStreamMapMatch(sampledTrajectory, dynamicWeightAdjuster, windowSize);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSamepleRate);
                    }

                    resultLogStream.println("Accuracy: " + EvaluateUtils.getTotalAcc());

                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    resultLogStream.println("backtrack num: " + ourMapMatcher.getDelayNums());
                    resultLogStream.println("backtrack time: " + ourMapMatcher.getDelayTime());
                    resultLogStream.println("average backtrack time: " + ourMapMatcher.getDelayTime() / ourMapMatcher.getDelayNums());

                    EvaluateUtils.reset();
                    totalDelay = 0;
                }


                // base onlineHmm
                if (BASE) {
                    resultLogStream.println("---- Base onlineHmm ---- sampleRate: " + resultSamepleRate);
                    indexLogStream.println("---- Base onlineHmm ---- sampleRate: " + resultSamepleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSamepleRate);
                        FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        long startTime = System.nanoTime();
                        MapMatchedTrajectory result = baseMapMatcher.streamMapMatch(sampledTrajectory, fixedWeightAdjuster);
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSamepleRate);
                    }

                    resultLogStream.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

                    averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                    resultLogStream.println("Average Delay: " + averageDelay + " ms");

                    EvaluateUtils.reset();
                    totalDelay = 0;
                }


                // AMM
                if (AMM) {
                    resultLogStream.println("---- AMM ---- sampleRate: " + resultSamepleRate);
                    indexLogStream.println("---- AMM ---- sampleRate: " + resultSamepleRate);
                    for (int index = startIndex; index < testNum; index++) {
                        trajectory = ModelGenerator.generateTrajectory(index);
                        Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSamepleRate);

                        indexLogStream.println("Trajectory index: " + index);

                        // offline hmm(label)
                        MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                        long startTime = System.nanoTime();
                        ammMapMatcher.mapMatch(sampledTrajectory, index);
                        if (ammMapMatcher.getMatchedList() == null) {
                            continue;
                        }
                        MapMatchedTrajectory result = convertMatchedListToTrajectory(ammMapMatcher.getMatchedList(), trajectory.getTid(), trajectory.getOid());
                        long endTime = System.nanoTime();
                        long delay = endTime - startTime;
                        totalDelay += delay;
                        EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSamepleRate);
                    }

                resultLogStream.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

                averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                resultLogStream.println("Average Delay: " + averageDelay + " ms");

                EvaluateUtils.reset();
                totalDelay = 0;
            }


            // AOMM
            if (AOMM) {
                resultLogStream.println("---- AOMM ---- sampleRate: " + resultSamepleRate);
                indexLogStream.println("---- AOMM ---- sampleRate: " + resultSamepleRate);
                for (int index = startIndex; index < testNum; index++) {
                    trajectory = ModelGenerator.generateTrajectory(index);
                    Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSamepleRate);

                    indexLogStream.println("Trajectory index: " + index);

                    // offline hmm(label)
                    MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                    long startTime = System.nanoTime();
                    MapMatchedTrajectory result = aommMapMatcher.aommMapMatch(sampledTrajectory);
                    long endTime = System.nanoTime();
                    long delay = endTime - startTime;
                    totalDelay += delay;
                    EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSamepleRate);
                }

                resultLogStream.println("Accuracy: "+ EvaluateUtils.getTotalAcc());
                resultLogStream.println("pointNums: " + EvaluateUtils.getTotalNum());

                averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
                resultLogStream.println("Average Delay: " + averageDelay + " ms");

                EvaluateUtils.reset();
                totalDelay = 0;
            }


            // DW-RMM
            if (DWRMM) {
                resultLogStream.println("---- DW-RMM ---- sampleRate: " + resultSamepleRate);
                indexLogStream.println("---- DW-RMM ---- sampleRate: " + resultSamepleRate);
                for (int index = startIndex; index < testNum; index++) {
                    trajectory = ModelGenerator.generateTrajectory(index);
                    Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, originalSampleRate, resultSamepleRate);

                    indexLogStream.println("Trajectory index: " + index);

                    // offline hmm(label)
                    MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                    long startTime = System.nanoTime();
                    MapMatchedTrajectory result = dwrmmMapMatcher.dwrmmMapMatch(sampledTrajectory);
                    long endTime = System.nanoTime();
                    long delay = endTime - startTime;
                    totalDelay += delay;
                    EvaluateUtils.getAccuracy(labelResult, result, originalSampleRate, resultSamepleRate);
                }

                resultLogStream.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

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
}
