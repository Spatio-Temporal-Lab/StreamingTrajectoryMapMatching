import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.Candidate;
import org.urbcomp.cupid.db.algorithm.mapmatch.aomm.AommMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.dwrmm.DwrmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
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

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class experiment {
    private Trajectory trajectory;
    private TiHmmMapMatcher labelMapMatcher;
    private StreamMapMatcher ourMapMatcher;
    private StreamMapMatcher baseMapMatcher;
    private AmmMapMatcher ammMapMatcher;
    private AommMapMatcher aommMapMatcher;
    private DwrmmMapMatcher dwrmmMapMatcher;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    @Before
    public void setUp() throws IOException {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        labelMapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        ourMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        baseMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        ammMapMatcher = new AmmMapMatcher(roadNetwork);
        aommMapMatcher = new AommMapMatcher(roadNetwork);
        dwrmmMapMatcher = new DwrmmMapMatcher(roadNetwork);
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void accuracyAndEfficiencyTest() throws AlgorithmExecuteException {
        long totalDelay = 0; // 总延迟，单位为纳秒
        double averageDelay;
        int startIndex = 1;
        int windowSize = 10;
        int testNum = 2000;
        boolean OURS = true;
        boolean BASE = true;
        boolean AMM = true;
        boolean AOMM = true;
        boolean DWRMM = true;

        // our method
        if (OURS) {
            System.out.println("---- OURS ----");
            for (int index = startIndex; index <= testNum; index++) {
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, 3, 3);
                DynamicWeightAdjuster dynamicWeightAdjuster = new DynamicWeightAdjuster();
                FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                // offline hmm(label)
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                // 计算准确率和延迟
                long startTime = System.nanoTime();
                MapMatchedTrajectory result = ourMapMatcher.onlineStreamMapMatch(sampledTrajectory, dynamicWeightAdjuster, windowSize);
                long endTime = System.nanoTime();
                long delay = endTime - startTime;
                totalDelay += delay;
                EvaluateUtils.getAccuracy(labelResult, result, 3, 3);

//                System.out.println("Index: " + index);
//                System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
//                System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
//                System.out.println();
            }
            // 准确率
            System.out.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

            // 平均延迟
            averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
            System.out.println("Average Delay: " + averageDelay + " ms");

            EvaluateUtils.reset();
            totalDelay = 0;
        }


        // base onlineHmm
        if (BASE) {
            System.out.println("---- Base onlineHmm ----");
            for (int index = startIndex; index <= testNum; index++) {
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index);
                FixedWeightAdjuster fixedWeightAdjuster = new FixedWeightAdjuster();

                // offline hmm(label)
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                // 计算准确率和延迟
                long startTime = System.nanoTime();
                MapMatchedTrajectory result = baseMapMatcher.streamMapMatch(sampledTrajectory, fixedWeightAdjuster);
                long endTime = System.nanoTime();
                long delay = endTime - startTime;
                totalDelay += delay;
                EvaluateUtils.getAccuracy(labelResult, result);

//                System.out.println("Index: " + index);
//                System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
//                System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
//                System.out.println();
            }
            // 准确率
            System.out.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

            // 平均延迟
            averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
            System.out.println("Average Delay: " + averageDelay + " ms");

            EvaluateUtils.reset();
            totalDelay = 0;
        }


        // AMM
        if (AMM) {
            System.out.println("---- AMM ----");
            for (int index = 1; index <= testNum; index++) {
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index);

                // offline hmm(label)
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                // 计算准确率和延迟
                long startTime = System.nanoTime();
                ammMapMatcher.mapMatch(sampledTrajectory, index);
                MapMatchedTrajectory result = convertMatchedListToTrajectory(ammMapMatcher.getMatchedList(), trajectory.getTid(), trajectory.getOid());
                long endTime = System.nanoTime();
                long delay = endTime - startTime;
                totalDelay += delay;
                EvaluateUtils.getAccuracy(labelResult, result);

//                System.out.println("Index: " + index);
//                System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
//                System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
//                System.out.println();
            }
            // 准确率
            System.out.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

            // 平均延迟
            averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
            System.out.println("Average Delay: " + averageDelay + " ms");

            EvaluateUtils.reset();
            totalDelay = 0;
        }


        // AOMM
        if (AOMM) {
            System.out.println("---- AOMM ----");
            for (int index = 1; index <= testNum; index++) {
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index);

                // offline hmm(label)
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                // 计算准确率和延迟
                long startTime = System.nanoTime();
                MapMatchedTrajectory result = aommMapMatcher.aommMapMatch(sampledTrajectory);
                long endTime = System.nanoTime();
                long delay = endTime - startTime;
                totalDelay += delay;
                EvaluateUtils.getAccuracy(labelResult, result);

//                System.out.println("Index: " + index);
//                System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
//                System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
//                System.out.println();
            }
            // 准确率
            System.out.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

            // 平均延迟
            averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
            System.out.println("Average Delay: " + averageDelay + " ms");

            EvaluateUtils.reset();
            totalDelay = 0;
        }


        // DW-RMM
        if (DWRMM) {
            System.out.println("---- DW-RMM ----");
            for (int index = 1; index <= testNum; index++) {
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index);

                // offline hmm(label)
                MapMatchedTrajectory labelResult = labelMapMatcher.mapMatch(trajectory);

                // 计算准确率和延迟
                long startTime = System.nanoTime();
                MapMatchedTrajectory result = dwrmmMapMatcher.dwrmmMapMatch(sampledTrajectory);
                long endTime = System.nanoTime();
                long delay = endTime - startTime;
                totalDelay += delay;
                EvaluateUtils.getAccuracy(labelResult, result);

//                System.out.println("Index: " + index);
//                System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
//                System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
//                System.out.println();
            }
            // 准确率
            System.out.println("Accuracy: "+ EvaluateUtils.getTotalAcc());

            // 平均延迟
            averageDelay = (double) totalDelay / EvaluateUtils.getTotalNum() / 1_000_000.0;
            System.out.println("Average Delay: " + averageDelay + " ms");
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
