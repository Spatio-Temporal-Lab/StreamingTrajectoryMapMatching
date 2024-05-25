
package org.urbcomp.cupid.db.algorithm.predict;

import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;

import java.util.List;
import java.util.ArrayList;
import java.io.IOException;

public class PredictTest {
    public static void main(String[] args) {
        String observationsPath = "data/output.txt";
        int sequenceLength = 12;
        DataPreparation.createTrainingData(observationsPath, sequenceLength);
        // 初始化LSTM预测模型
        LSTMModel predictor = new LSTMModel(2, 2, 100); // 输入维度为3（纬度、经度、时间），输出维度为2（匹配点纬度、经度），LSTM层大小为50
        try {
            predictor.loadModel("best_lstm_model.zip");
        } catch (IOException e) {
            e.printStackTrace();
        }

        Trajectory trajectory = ModelGenerator.generateTrajectory();
        List<GPSPoint> pointList = trajectory.getGPSPointList().subList(0,12);
        GPSPoint label = trajectory.getGPSPointList().get(12);
        double[][][] input = new double[1][2][12];
        for (int i = 0; i < 12; i++){
            input[0][0][i] = pointList.get(i).getLat();
            input[0][1][i] = pointList.get(i).getLng();
//            input[0][2][i] = pointList.get(i).getTime().getTime();
        }
        long start = System.nanoTime();
        double[][] predictedPositions = predictor.predict(input);
        long end = System.nanoTime();
        System.out.println((end -start)/1_000_000.0);
        // 取最后一个时间步的预测值
        System.out.println(DataPreparation.denormalizeLng(predictedPositions[0][1]) + " " + DataPreparation.denormalizeLat(predictedPositions[0][0]));
        System.out.println(label);
    }

}