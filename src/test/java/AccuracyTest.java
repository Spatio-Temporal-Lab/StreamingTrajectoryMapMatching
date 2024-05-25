import org.junit.Test;
import org.urbcomp.cupid.db.util.EvaluateUtils;

public class AccuracyTest {
    @Test
    public void testAccuracy() {
        String labelPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_stream.geojson";
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath);
        System.out.println("Accuracy:" + accuracy);
    }
}
