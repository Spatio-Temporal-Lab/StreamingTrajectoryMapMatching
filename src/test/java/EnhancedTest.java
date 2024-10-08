import org.junit.Before;
import org.junit.Test;
import org.urbcomp.cupid.db.algorithm.history.generateHistoryProb;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.BidirectionalManyToManyShortestPath;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.EvaluateUtils;
import org.xml.sax.SAXException;

import javax.xml.bind.JAXBException;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class EnhancedTest {

    private Trajectory trajectory;
    private TiHmmMapMatcher mapMatcher;
    private StreamMapMatcher mapMatcher2;
    //    private ShortestPathPathRecover recover;
    private RoadNetwork roadNetwork;

    @Before
    public void setUp() {
        trajectory = ModelGenerator.generateTrajectory();
        roadNetwork = ModelGenerator.generateRoadNetwork();
        mapMatcher = new TiHmmMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork));
//        recover = new ShortestPathPathRecover(roadNetwork, new BiDijkstraShortestPath(roadNetwork));
    }

    @Test
    public void enhancedAccTest() throws AlgorithmExecuteException, IOException, JAXBException, SAXException {
        String basePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\";
        String trajectoryWritePath = basePath + "trajectory\\";
        String labelMapMatcherWritePath = basePath + "match\\label\\";
        String streamMapMatcherWritePath = basePath + "match\\stream\\";
        String csvWritePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\csv\\enhancedAccTest-Cache.csv";
        boolean isFirst = true;

        int testNum = 100;
        int startIndex = 1;
        testNum += startIndex;

        int[] sampleRates = {0, 5, 10};

//        int sampleRate = 0;

        List<TestResult> testResults = new ArrayList<>(); // 用于存储实验结果

        for (int sampleRate : sampleRates) {
            System.out.println("Testing with sample rate: " + sampleRate);
            for (int index = startIndex; index < testNum; index++) {
                System.out.println("Index: " + index);
                csvWritePath = "C:\\Users\\t1anyu\\Desktop\\Results\\MapMatching\\sample\\enhancedAccTest-Cache-sample" + sampleRate + ".csv";
                // 单向 dijkstra
//            mapMatcher2 = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), historyProb);
                // 双向 dijkstra
                mapMatcher2 = new StreamMapMatcher(roadNetwork, new SimpleManyToManyShortestPath(roadNetwork), new BidirectionalManyToManyShortestPath(roadNetwork));

                // 创建轨迹
                trajectory = ModelGenerator.generateTrajectory(index);
                Trajectory sampledTrajectory = ModelGenerator.generateTrajectory(index, sampleRate);

                // 写入轨迹
//            BufferedWriter writer = new BufferedWriter(new FileWriter(trajectoryWritePath + "trajectory_" + index + ".json"));
//            writer.write(sampledTrajectory.toGeoJSON());

                // 进行轨迹匹配并记录匹配结果
                MapMatchedResult mapMatchedResult = mapMatchTrajectory(trajectory, index, labelMapMatcherWritePath, false);
                MapMatchedResult mapMatchedResult2 = streamMapMatchTrajectory(sampledTrajectory, index, streamMapMatcherWritePath, false);

                MapMatchedTrajectory mmTrajectory = mapMatchedResult.getMmTrajectory();
                MapMatchedTrajectory mmTrajectory2 = mapMatchedResult2.getMmTrajectory();

                // 计算准确率
                EvaluateUtils.getAccuracy(mmTrajectory, mmTrajectory2, sampleRate);
                double accuracy = EvaluateUtils.getCurrAcc();

                // 统计实验信息
                double accNum = accuracy * mmTrajectory2.getMmPtList().size();
                testResults.add(new TestResult(accuracy, accNum));

                // 计算当前及平均准确率
                double avgAcc = calculateAverageAccuracy(testResults);

                // 输出结果
                System.out.println("currAcc: " + accuracy);
                System.out.println("averageAcc: " + avgAcc);
                System.out.println("Label Time taken: " + mapMatchedResult.getTime() + " seconds");
                System.out.println("Stream Time taken: " + mapMatchedResult2.getTime() + " seconds");
                System.out.println();

                // 保存实验信息到 CSV 文件
                saveToCSV(index, accuracy, avgAcc, mapMatchedResult.getTime(), mapMatchedResult2.getTime(), csvWritePath, isFirst);
                isFirst = false;
//            writer.close();
            }
            isFirst = true;
        }
    }

    private MapMatchedResult mapMatchTrajectory(Trajectory trajectory, int index, String writePath, boolean isWrite) throws IOException, AlgorithmExecuteException {

        // 进行匹配
        long startTime = System.currentTimeMillis();
        MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory);
        long endTime = System.currentTimeMillis();
        double elapsedTime = (endTime - startTime) / 1000.0;

        if (isWrite) {
            BufferedWriter writer = new BufferedWriter(new FileWriter(writePath + "label_" + index + ".json"));
            writer.write(mmTrajectory.toGeoJSON());
            writer.close();
        }

        return new MapMatchedResult(mmTrajectory, elapsedTime);
    }

    private MapMatchedResult streamMapMatchTrajectory(Trajectory sampledTrajectory, int index, String writePath, boolean isWrite) throws IOException, AlgorithmExecuteException, JAXBException, org.xml.sax.SAXException {
        // 进行流式匹配
        long startTime = System.currentTimeMillis();
        MapMatchedTrajectory mmTrajectory2 = mapMatcher2.streamMapMatch(sampledTrajectory);
        long endTime = System.currentTimeMillis();
        double elapsedTime = (endTime - startTime) / 1000.0;

        assert mmTrajectory2 != null;

        if (isWrite) {
            BufferedWriter writer = new BufferedWriter(new FileWriter(writePath + "stream_" + index + ".json"));
            writer.write(mmTrajectory2.toGeoJSON());
            writer.close();
        }

        return new MapMatchedResult(mmTrajectory2, elapsedTime);
    }

    private double calculateAverageAccuracy(List<TestResult> testResults) {
        double totalAcc = 0;
        for (TestResult result : testResults) {
            totalAcc += result.getAccuracy();
        }
        return totalAcc / testResults.size();
    }

    private void saveToCSV(int index, double currAcc, double avgAcc, double labelTime, double streamTime, String writePath, boolean isFirst) throws IOException {
        BufferedWriter writer;
        if (isFirst) {
            writer = new BufferedWriter(new FileWriter(writePath, false));
            writer.write("index,currAcc,avgAcc,labelTime,streamTime\n");
            writer.write(index + "," + currAcc + "," + avgAcc + "," + labelTime + "," + streamTime + "\n");
        } else {
            writer = new BufferedWriter(new FileWriter(writePath, true));
            writer.write(index + "," + currAcc + "," + avgAcc + "," + labelTime + "," + streamTime + "\n");
        }
        writer.close();
    }
}

class TestResult {
    private final double accuracy;
    private final double accNum;

    public TestResult(double accuracy, double accNum) {
        this.accuracy = accuracy;
        this.accNum = accNum;
    }

    public double getAccuracy() {
        return accuracy;
    }

    public double getAccNum() {
        return accNum;
    }
}

class MapMatchedResult {
    private final MapMatchedTrajectory mmTrajectory;
    private final double time;


    MapMatchedResult(MapMatchedTrajectory mmTrajectory, double time) {
        this.mmTrajectory = mmTrajectory;
        this.time = time;
    }

    public MapMatchedTrajectory getMmTrajectory() {
        return mmTrajectory;
    }

    public double getTime() {
        return time;
    }
}
