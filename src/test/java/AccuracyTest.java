import org.junit.Test;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import javax.swing.*;

public class AccuracyTest {
    @Test
    public void testAccuracy() {
        String labelPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_stream_s0.geojson";
        int sampleRate = 0;
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath, sampleRate);
        System.out.println("Accuracy:" + accuracy);
    }
}
