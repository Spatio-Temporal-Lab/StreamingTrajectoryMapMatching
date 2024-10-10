import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * Test class for the OnlineMapMatcher functionality, including tests for
 * trajectory matching, accuracy evaluation, and algorithm correctness.
 */
public class OnlineMapMatcherTest {

    private Trajectory trajectory;
    private StreamMapMatcher streamMapMatcher;
    private TiHmmMapMatcher baseMapMatcher;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    /**
     * Sets up the test environment by generating a sample trajectory and road network.
     * Initializes the base and stream map matchers.
     */
    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        baseMapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        streamMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    /**
     * Tests whether the online Viterbi algorithm can run successfully
     * for a single trajectory.
     *
     * @throws AlgorithmExecuteException if the algorithm encounters an execution error
     * @throws IOException if there are I/O issues during the test
     */
    @Test
    public void matchSingleTrajectory() throws AlgorithmExecuteException, IOException {
        trajectory = ModelGenerator.generateTrajectory(6);
        MapMatchedTrajectory mmTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory);

        System.out.println(trajectory.toGeoJSON());
        System.out.println(mmTrajectory.toGeoJSON());

        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);

        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }

    /**
     * Tests the correctness of the online Viterbi algorithm for multiple trajectories.
     * Note: This test requires that the related convergence status check in
     * [computeViterbiSequence] be commented out.
     *
     * @throws AlgorithmExecuteException if the algorithm encounters an execution error
     */
    @Test
    public void matchMultiTrajectory() throws AlgorithmExecuteException {
        for (int i = 1; i <= 10; i++) {
            System.out.println("------------------------------");
            System.out.println("index: " + i);
            System.out.println("------------------------------");
            trajectory = ModelGenerator.generateTrajectory(i);

            MapMatchedTrajectory mmTrajectory = streamMapMatcher.streamMapMatch(trajectory);
            MapMatchedTrajectory onlineMMTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory);

            List<MapMatchedPoint> mmPtList = mmTrajectory.getMmPtList();
            List<MapMatchedPoint> onlineMMPtList = onlineMMTrajectory.getMmPtList();

            assertEquals(mmPtList.size(), onlineMMPtList.size());
        }
    }

    /**
     * Tests whether the online Viterbi algorithm improves accuracy in
     * trajectory matching.
     *
     * @throws AlgorithmExecuteException if the algorithm encounters an execution error
     */
    @Test
    public void onlineMatchAccuracy() throws AlgorithmExecuteException {
        int testNum = 100;
        int sampleRate = 0;
        for (int i = 1; i <= testNum; i++) {
            System.out.println("===========================");
            System.out.println("index: " + i);
            System.out.println("===========================");
            trajectory = ModelGenerator.generateTrajectory(i);
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

            assert sampledTrajectory != null;

            MapMatchedTrajectory baseMapMatchedTrajectory = baseMapMatcher.mapMatch(trajectory);
            MapMatchedTrajectory streamOnlineMapMatchedTrajectory = streamMapMatcher.streamMapMatch(sampledTrajectory);

            assert baseMapMatchedTrajectory.getMmPtList().size() == streamOnlineMapMatchedTrajectory.getMmPtList().size();

            EvaluateUtils.getAccuracy(baseMapMatchedTrajectory, streamOnlineMapMatchedTrajectory, sampleRate);

            System.out.println("===========================");
            System.out.println("results: ");
            System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
            System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
            System.out.println("pointNum: " + EvaluateUtils.getTotalNum());
            System.out.println("===========================");

            System.out.println();
        }
    }

    /**
     * Tests the accuracy of trajectory matching using non-online streamMatch.
     * Records matching results for each trajectory and generates two CSV files.
     */
    @Test
    public void testNoOnlineStreamMatch() throws AlgorithmExecuteException {
        int testNum = 100;
        int sampleRate = 0;

        // Create CSV file for online results
        try (PrintWriter streamWriter = new PrintWriter(new FileWriter("streamResult.csv"))) {

            // Write headers for the CSV file
            streamWriter.println("TrajectoryIndex,currPointNum,totalPointNum,currAcc,totalAcc");

            for (int i = 1; i <= testNum; i++) {
                System.out.println("===========================");
                System.out.println("index: " + i);
                System.out.println("===========================");

                trajectory = ModelGenerator.generateTrajectory(i);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

                assert sampledTrajectory != null;

                // Match using baseMatch
                MapMatchedTrajectory baseMapMatchedTrajectory = baseMapMatcher.mapMatch(trajectory);
                // Match using onlineStreamMatch
                MapMatchedTrajectory streamMapMatchedTrajectory = streamMapMatcher.streamMapMatch(sampledTrajectory);

                // Record number of matched points
                int streamCurrPointNum = streamMapMatchedTrajectory.getMmPtList().size();
                System.out.println("size: " + streamCurrPointNum);

                // Evaluate accuracy for onlineStreamMatch
                EvaluateUtils.getAccuracy(baseMapMatchedTrajectory, streamMapMatchedTrajectory, sampleRate);
                double streamCurrAcc = EvaluateUtils.getCurrAcc();
                double streamTotalAcc = EvaluateUtils.getTotalAcc();
                int streamTotalPointNum = EvaluateUtils.getTotalNum();

                // Write results to CSV
                streamWriter.printf("%d,%d,%d,%.4f,%.4f%n", i, streamCurrPointNum, streamTotalPointNum, streamCurrAcc, streamTotalAcc);

                System.out.println("===========================");
                System.out.println("Stream results: ");
                System.out.println("currAcc: " + streamCurrAcc);
                System.out.println("totalAcc: " + streamTotalAcc);
                System.out.println("pointNum: " + streamTotalPointNum);
                System.out.println("===========================");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Tests the accuracy of trajectory matching using onlineStreamMatch.
     * Records matching results for each trajectory and generates a CSV file.
     */
    @Test
    public void testOnlineStreamMatch() throws AlgorithmExecuteException {
        int testNum = 100;
        int sampleRate = 0;

        // Create CSV file for online results
        try (PrintWriter onlineWriter = new PrintWriter(new FileWriter("onlineStreamResult.csv"))) {

            // Write headers for the CSV file
            onlineWriter.println("TrajectoryIndex,currPointNum,totalPointNum,currAcc,totalAcc");

            for (int i = 1; i <= testNum; i++) {
                System.out.println("===========================");
                System.out.println("index: " + i);
                System.out.println("===========================");

                trajectory = ModelGenerator.generateTrajectory(i);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

                assert sampledTrajectory != null;

                // Match using baseMatch
                MapMatchedTrajectory baseMapMatchedTrajectory = baseMapMatcher.mapMatch(trajectory);
                // Match using onlineStreamMatch
                MapMatchedTrajectory onlineMapMatchedTrajectory = streamMapMatcher.onlineStreamMapMatch(sampledTrajectory);

                // Record number of matched points
                int onlineCurrPointNum = onlineMapMatchedTrajectory.getMmPtList().size();
                System.out.println("size: " + onlineCurrPointNum);

                // Evaluate accuracy for onlineStreamMatch
                EvaluateUtils.getAccuracy(baseMapMatchedTrajectory, onlineMapMatchedTrajectory, sampleRate);
                double onlineCurrAcc = EvaluateUtils.getCurrAcc();
                double onlineTotalAcc = EvaluateUtils.getTotalAcc();
                int onlineTotalPointNum = EvaluateUtils.getTotalNum();

                // Write results to CSV
                onlineWriter.printf("%d,%d,%d,%.4f,%.4f%n", i, onlineCurrPointNum, onlineTotalPointNum, onlineCurrAcc, onlineTotalAcc);

                System.out.println("===========================");
                System.out.println("Online results: ");
                System.out.println("currAcc: " + onlineCurrAcc);
                System.out.println("totalAcc: " + onlineTotalAcc);
                System.out.println("pointNum: " + onlineTotalPointNum);
                System.out.println("===========================");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
