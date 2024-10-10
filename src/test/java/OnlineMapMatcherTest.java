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

import java.io.IOException;
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
        int testNum = 25;
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
}
