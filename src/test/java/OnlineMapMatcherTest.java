import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
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

public class OnlineMapMatcherTest {
    private Trajectory trajectory;
    private StreamMapMatcher streamMapMatcher;
    private TiHmmMapMatcher baseMapMatcher;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        baseMapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
        streamMapMatcher = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    /**
     * 测试在线维特比算法能否正常运行
     */
    @Test
    public void matchSingleTrajectory() throws AlgorithmExecuteException, IOException {
        MapMatchedTrajectory mmTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory);
        System.out.println(trajectory.toGeoJSON());
        System.out.println(mmTrajectory.toGeoJSON());
        assertEquals(trajectory.getGPSPointList().size(), mmTrajectory.getMmPtList().size());
        List<PathOfTrajectory> pTrajectories = recover.recover(mmTrajectory);
        System.out.println(pTrajectories.get(0).toGeoJSON());
        assertEquals(1, pTrajectories.size());
    }



    /**
     * Note: 测试该函数需要注释掉[computeViterbiSequence]中有关收敛状态的[else if]语句
     * 测试在线维特比算法的正确性
     */
    @Test
    public void matchMultiTrajectory() throws AlgorithmExecuteException {
        for (int i = 1; i <= 6; i++) {
            System.out.println("------------------------------");
            System.out.println("index: " + i);
            System.out.println("------------------------------");
            trajectory = ModelGenerator.generateTrajectory(i);

            MapMatchedTrajectory mmTrajectory = streamMapMatcher.streamMapMatch(trajectory);
            MapMatchedTrajectory onlineMMTrajectory = streamMapMatcher.onlineStreamMapMatch(trajectory);

            List<MapMatchedPoint> mmPtList = mmTrajectory.getMmPtList();
            List<MapMatchedPoint> onlineMMPtList = onlineMMTrajectory.getMmPtList();

            assertEquals(mmPtList.size(), onlineMMPtList.size());

            for (int j = 0; j < mmPtList.size(); j++) {
                MapMatchedPoint mmPoint = mmPtList.get(j);
                MapMatchedPoint onlineMMPoint = onlineMMPtList.get(j);
                assertEquals(mmPoint.getCandidatePoint().getLat(), onlineMMPoint.getCandidatePoint().getLat(), 1e-6);
                assertEquals(mmPoint.getCandidatePoint().getLng(), onlineMMPoint.getCandidatePoint().getLng(), 1e-6);
                assertEquals(mmPoint.getTime(), onlineMMPoint.getTime());
            }
        }
    }

    /**
     * 测试在线维特比算法能否提高准确率
     */
    @Test
    public void onlineMatchAccuracy() throws AlgorithmExecuteException {
        int testNum = 6;
        int sampleRate = 0;
        for (int i = 1; i <= testNum; i++) {
            System.out.println("===========================");
            System.out.println("index: " + i);
            System.out.println("===========================");
            trajectory = ModelGenerator.generateTrajectory(i);
            Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(i, sampleRate);

            assert sampledTrajectory != null;

            MapMatchedTrajectory baseMapMatchedTrajectory = baseMapMatcher.mapMatch(trajectory);
            MapMatchedTrajectory streamOnlineMapMatchedTrajectory = streamMapMatcher.onlineStreamMapMatch(sampledTrajectory);

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
