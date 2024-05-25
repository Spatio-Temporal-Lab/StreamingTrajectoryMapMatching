package org.urbcomp.cupid.db.algorithm.predict;

import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;

import java.io.IOException;

public class TrainModel {
    public static void main(String[] args) throws IOException {
        String observationsPath = "data/output.txt";
        double bestValidationLoss = Double.MAX_VALUE;
        int sequenceLength = 12;
        int numEpochs = 5000;

        DataSetIterator trainData = DataPreparation.createTrainingData(observationsPath, sequenceLength);
        System.out.println(trainData.next());

        LSTMModel predictor = new LSTMModel(2, 2, 100);

        long startTime = System.currentTimeMillis();

        for (int i = 0; i < numEpochs; i++) {
            double score = predictor.train(trainData);
            System.out.println("Epoch " + (i + 1) + " complete. Loss: " + score);
            if (score < bestValidationLoss) {
                bestValidationLoss = score;
                predictor.saveModel("best_lstm_model.zip");  // 保存最优模型
            }
        }

        long endTime = System.currentTimeMillis();
        long duration = (endTime - startTime) / 1000;

        System.out.println("Training complete. Time taken: " + duration + " seconds.");

        try {
            predictor.saveModel("lstm_model.zip");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}