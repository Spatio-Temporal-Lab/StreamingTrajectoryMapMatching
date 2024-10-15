import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;
import org.urbcomp.cupid.db.util.GeoJSONParser;

import java.io.*;
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
        int windowSize = 10;
        trajectory = ModelGenerator.generateTrajectory(6);
        MapMatchedTrajectory mmTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory, windowSize);

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
        int windowSize = 10;
        for (int i = 1; i <= 10; i++) {
            System.out.println("------------------------------");
            System.out.println("index: " + i);
            System.out.println("------------------------------");
            trajectory = ModelGenerator.generateTrajectory(i);

            MapMatchedTrajectory mmTrajectory = streamMapMatcher.streamMapMatch(trajectory);
            MapMatchedTrajectory onlineMMTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory, windowSize);

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
    public void onlineMatchAccuracy() throws AlgorithmExecuteException, IOException {
        int testNum = 3;
        int sampleRate = 0;
        int windowSize = 50;

        for (int i = 1; i <= testNum; i++) {
            System.out.println("*********************************************************");
            System.out.println("index: " + i);
            System.out.println("*********************************************************");
            System.out.println();

            trajectory = ModelGenerator.generateTrajectory(i);
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

            assert sampledTrajectory != null;

            MapMatchedTrajectory baseMapMatchedTrajectory = baseMapMatcher.mapMatch(trajectory);
            MapMatchedTrajectory streamOnlineMapMatchedTrajectory = streamMapMatcher.onlineStreamMapMatch(sampledTrajectory, windowSize);

            assert baseMapMatchedTrajectory.getMmPtList().size() == streamOnlineMapMatchedTrajectory.getMmPtList().size();

            EvaluateUtils.getAccuracy(baseMapMatchedTrajectory, streamOnlineMapMatchedTrajectory, sampleRate);

            System.out.println("*********************************************************");
            System.out.println("results: ");
            System.out.println("currAcc: " + EvaluateUtils.getCurrAcc());
            System.out.println("totalAcc: " + EvaluateUtils.getTotalAcc());
            System.out.println("pointNum: " + EvaluateUtils.getTotalNum());
            System.out.println("*********************************************************");

            System.out.println();

//            String outputFile = "trajectory_" + i + ".geojson";
//            BufferedWriter writer = new BufferedWriter(new FileWriter(outputFile));
//            writer.write(baseMapMatchedTrajectory.toGeoJSON(true));
//            writer.flush();
//            writer.close();
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
        int windowSize = 10;

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
                MapMatchedTrajectory onlineMapMatchedTrajectory = streamMapMatcher.onlineStreamMapMatch(sampledTrajectory, windowSize);

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

    @Test
    public void testConvergedSequenceAccuracy() throws Exception {
        int testNum = 1; // Number of trajectories to test
        int sampleRate = 0; // Sample rate for trajectory generation
        int windowSize = 10;
        double epsilon = 1e-6; // Allowable error for latitude/longitude comparison

        for (int i = 1; i <= testNum; i++) {
            System.out.println("===========================");
            System.out.println("Testing trajectory index: " + i);
            System.out.println("===========================");

            // Generate the trajectory and perform online map matching
            trajectory = ModelGenerator.generateTrajectory(i);
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

            assert sampledTrajectory != null;

            // Perform online map matching
            MapMatchedTrajectory streamOnlineMapMatchedTrajectory = streamMapMatcher.onlineStreamMapMatch(sampledTrajectory, windowSize);

            // Get converged sequence from the streamMapMatcher
            List<SequenceState> convergedSequence = streamMapMatcher.convergedSequence;

            // Load the corresponding GeoJSON file
            String geojsonFilePath = "trajectory_" + i + ".geojson";

            // Parse the GeoJSON to extract rawPoints
            List<GPSPoint> rawPointsFromGeoJSON = GeoJSONParser.parseRawPointsFromGeoJSON(geojsonFilePath);
            List<CandidatePoint> candidatePointsFromGeoJSON = GeoJSONParser.parseCandidatePointsFromGeoJSON(geojsonFilePath);

            // Ensure the sizes match
            System.out.println("Converged sequence size: " + convergedSequence.size());
            System.out.println("Global sequence size: " + rawPointsFromGeoJSON.size());

            // Use two pointers to compare raw points
            int seqPointer = 0;
            int geoPointer = 0;

            while (seqPointer < convergedSequence.size() && geoPointer < rawPointsFromGeoJSON.size()) {
                SequenceState sequenceState = convergedSequence.get(seqPointer);

                GPSPoint rawPointFromSequence = sequenceState.getObservation();
                CandidatePoint candidatePointFromSequence = sequenceState.getState();

                GPSPoint rawPointFromGeoJSON = rawPointsFromGeoJSON.get(geoPointer);
                CandidatePoint candidatePointFromGeoJSON = candidatePointsFromGeoJSON.get(geoPointer);

                // Compare raw points' latitude and longitude
                if (Math.abs(rawPointFromSequence.getLat() - rawPointFromGeoJSON.getLat()) < epsilon &&
                        Math.abs(rawPointFromSequence.getLng() - rawPointFromGeoJSON.getLng()) < epsilon) {

                    // If raw points match, compare the candidate points
                    boolean candidateMatch = false;
                    while (geoPointer < candidatePointsFromGeoJSON.size()) {
                        candidatePointFromGeoJSON = candidatePointsFromGeoJSON.get(geoPointer);

                        if (candidatePointFromSequence != null && candidatePointFromGeoJSON != null) {
                            // Compare candidate points
                            if (Math.abs(candidatePointFromSequence.getLat() - candidatePointFromGeoJSON.getLat()) < epsilon &&
                                    Math.abs(candidatePointFromSequence.getLng() - candidatePointFromGeoJSON.getLng()) < epsilon) {
                                candidateMatch = true;
                                break;
                            }
                        } else if (candidatePointFromSequence == null && candidatePointFromGeoJSON == null) {
                            // Both candidate points are null
                            candidateMatch = true;
                            break;
                        }

                        // If the candidate points do not match, move the geoPointer forward
                        geoPointer++;
                    }

                    // If a matching candidate point is found, move both pointers forward
                    if (candidateMatch) {
                        seqPointer++;
                        geoPointer++;
                    } else {
                        // No matching candidate point found for this GPS point
                        throw new AssertionError("Candidate point for GPS point at index " + seqPointer + " does not match.");
                    }
                } else {
                    // If raw points do not match, move the geoPointer forward
                    geoPointer++;
                }
            }

            // If all sequence points have been matched, the test passes
            if (seqPointer == convergedSequence.size()) {
                System.out.println("Trajectory " + i + " passed the accuracy test.");
            } else {
                System.out.println("Trajectory " + i + " did not pass the accuracy test.");
                throw new AssertionError("Converged sequence does not match the GeoJSON data.");
            }
        }
    }

}
