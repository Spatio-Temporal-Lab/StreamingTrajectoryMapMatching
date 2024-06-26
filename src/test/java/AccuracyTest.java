import org.junit.Test;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AccuracyTest {
    @Test
    public void testAccuracy() {
        String labelPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_stream.geojson";
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath);
        System.out.println("Accuracy:" + accuracy);
    }

    public void testPartAccuracy(int n) {
        String labelPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_stream.geojson";
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath, n);
        System.out.println("Accuracy:" + accuracy);
    }

    @Test
    public void testMapModify() {
        Map<Integer, testClass> map = new HashMap<>();
        map.put(1, new testClass("1", 1));
        map.put(2, new testClass("2", 2));
        map.put(3, new testClass("3", 3));
        System.out.println(map.get(1));
        testClass testClass = map.get(1);
        testClass.h = 2;
        System.out.println(map.get(1));
    }
}

class testClass {
    String s;
    int h;


    public testClass(String s, int h) {
        this.s = s;
        this.h = h;
    }

    @Override
    public String toString() {
        return "testClass{" +
                "s='" + s + '\'' +
                ", h=" + h +
                '}';
    }
}
