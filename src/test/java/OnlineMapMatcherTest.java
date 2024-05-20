import com.fasterxml.jackson.core.JsonProcessingException;
import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.onlinematch.OnlineMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi.OnlineViterbiDecoder;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi.StandardViterbiDecoder;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi.Auxiliary;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.OnlineNode;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayLinkedList;
import org.urbcomp.cupid.db.algorithm.mapmatch.routerecover.ShortestPathPathRecover;
import org.urbcomp.cupid.db.algorithm.shortestpath.BiDijkstraShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.PathOfTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;


import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class OnlineMapMatcherTest {
    private Trajectory trajectory;
    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    @Before
    public void setUpBeforeOnlineMapMatchTest() {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void testTwoWayLinkedList() {
        List<OnlineNode<Integer>> lists = new TwoWayLinkedList<>();
        lists.add(new OnlineNode<>(0, 0, 1, null));
        lists.add(new OnlineNode<>(0, 1, 1, null));
        lists.add(new OnlineNode<>(1, 0, 1, lists.get(0)));
        lists.add(new OnlineNode<>(1, 1, 1, lists.get(0)));
        OnlineNode<Integer> node = lists.get(2);
        System.out.println(node.toString());
        System.out.println(node.prev.toString());
        System.out.println(node.next.toString());
        System.out.println(node.parent.toString());
        assert node.parent == lists.get(0);
    }

    @Test
    public void testLinkedList() {
        List<Integer> linkList = new LinkedList<>();
        linkList.add(2);
        linkList.add(1);
        linkList.add(3);
        for (int i : linkList) {
            System.out.println(i);
        }
        List<Integer> arrList = new ArrayList<>();
        arrList.add(2);
        arrList.add(1);
        arrList.add(3);
        for (int i : arrList) {
            System.out.println(i);
        }
    }

    @Test
    public void testOnlineViterbiDecoder() {
        int K = 4, M = 4, T = 1000;

        double[][] A = {{0.96, 0.04, 0.0, 0.0},
                {0.0, 0.95, 0.05, 0.0},
                {0.0, 0.0, 0.85, 0.15},
                {0.1, 0.0, 0.0, 0.9}};

        double[][] E = {{0.6, 0.2, 0.0, 0.2},
                {0.1, 0.8, 0.1, 0.0},
                {0.0, 0.14, 0.76, 0.1},
                {0.1, 0.0, 0.1, 0.8}};

        double[] I = {0.25, 0.25, 0.25, 0.25};

        List<List<Double>> emissionProbs = Auxiliary.array2List(A);
        List<List<Double>> transitionProbs = Auxiliary.array2List(E);
        List<Double> initial = Auxiliary.array2List(I);

        int[] observation = new int[T];
        int previous = 0;
        OnlineViterbiDecoder omm = new OnlineViterbiDecoder(K, T);
        StandardViterbiDecoder smm = new StandardViterbiDecoder(K, T);

        omm.initialization(0, initial);

        for (int i = 0; i < 100 * 60 * 60; i++) {
            int count = i % T;
            observation[count] = (int) (previous + (2 * Math.random() % 2)) % K;
            previous = observation[count];
            omm.update(count, observation[count], emissionProbs, transitionProbs);

            if (count == T - 1) {
                omm.traceLastPart();
                smm.viterbi(observation, I, A, E);

                int minT = Math.min(T, 1000);
                int[] optimalPath = smm.getOptimalPath();
                List<Integer> decodedStream = omm.getDecodedStream();
                for (int p = 0; p < minT; p++) {
                    assertEquals(optimalPath[p], decodedStream.get(p),
                            "Mismatch at position " + p + ": expected " + optimalPath[p] + " but got " + decodedStream.get(p));
                }

                omm.initialization(0, initial);


            }
        }
    }

    @Test
    public void onlineMapMatchTest() throws JsonProcessingException {
        OnlineMapMatcher omm = new OnlineMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));

        MapMatchedTrajectory mmTraj = omm.mapMatch(trajectory);
        assert mmTraj.getMmPtList().size() == trajectory.getGPSPointList().size();
        System.out.println(mmTraj.toGeoJSON());

        List<PathOfTrajectory> pathOfTraj = recover.recover(mmTraj);
        System.out.println(pathOfTraj.get(0).toGeoJSON());
        assertEquals(pathOfTraj.size(), 1);

        omm.printNodeList();
//        omm.printStateList();
//        omm.printProbList();
        String filePath = "./src/main/resources/data/one-traj-test-online.json";
        try (FileWriter fileWriter = new FileWriter(filePath)) {
            fileWriter.write(pathOfTraj.get(0).toGeoJSON());
            System.out.println("PathOfTraj JSON saved to: " + filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
