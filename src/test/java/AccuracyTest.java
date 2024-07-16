import org.junit.Test;
import org.urbcomp.cupid.db.util.EvaluateUtils;

import java.io.*;

public class AccuracyTest {
    @Test
    public void testAccuracy() {
        String labelPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_labels.geojson";
        String streamPath = "D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\map_matched_trajectories_stream_s0.geojson";
        int sampleRate = 0;
        double accuracy = EvaluateUtils.calculateAccuracy(labelPath, streamPath, sampleRate);
        System.out.println("Accuracy:" + accuracy);
    }

    @Test
    public void compareCSVAccuracy() throws IOException {
        String file1Path = "src/main/resources/data/accuracy/stream/sample_rate/9/no-rectify.csv";
        String file2Path = "src/main/resources/data/accuracy/stream/sample_rate/9/rectify.csv";
        BufferedReader br1 = new BufferedReader(new FileReader(file1Path));
        BufferedReader br2 = new BufferedReader(new FileReader(file2Path));
        br1.readLine();
        br2.readLine();
        double file1TotalAccuracy = 0;
        int rowCount = 0;
        double file2TotalAccuracy = 0;
        String line1, line2;
        while ((line1 = br1.readLine()) != null && (line2 = br2.readLine()) != null) {
            String[] columns1 = line1.split(",");
            String[] columns2 = line2.split(",");
            double acc1 = Double.parseDouble(columns1[2]);
            double acc2 = Double.parseDouble(columns2[2]);
            System.out.print("trajectory " + (rowCount + 1));
            System.out.println(" delta accuracy: " + String.format("%.2f", (acc2 - acc1) * 100) + "%");
            file1TotalAccuracy += acc1;
            file2TotalAccuracy += acc2;
            rowCount++;
        }
        System.out.println("No Rectify: Average Accuracy: " + file1TotalAccuracy / rowCount);
        System.out.println("Rectify: Average Accuracy: " + file2TotalAccuracy / rowCount);
        br1.close();
        br2.close();
    }
}
