package org.urbcomp.cupid.db.algorithm.predict2;

import weka.classifiers.trees.RandomForest;
import weka.core.Instances;
import weka.core.SerializationHelper;

public class PredictTest {

    public static void main(String[] args) throws Exception {
        String observationsPath = "data/output.txt";
        Instances trainingData = DataPreparation.createTrainingData(observationsPath);
        long start = System.nanoTime();
        // 加载模型
        RandomForest loadedModel = (RandomForest) SerializationHelper.read("randomForestModel.model");
        // 示例预测
        double[] exampleFeatures = new double[]{30.69204, 104.04235, 12, 0, 0}; // 示例标准化特征
        Instances predictData = createPredictInstance(exampleFeatures, trainingData);
        double predictionLat = loadedModel.classifyInstance(predictData.firstInstance());
        double predictionLng = loadedModel.classifyInstance(predictData.firstInstance());
        long end = System.nanoTime();
        System.out.println((end - start)/1000000);

        System.out.println("Predicted Latitude: " + DataPreparation.denormalizeLat(predictionLat));
        System.out.println("Predicted Longitude: " + DataPreparation.denormalizeLng(predictionLng));
    }

    private static Instances createPredictInstance(double[] features, Instances trainingData) {
        Instances predictData = new Instances(trainingData, 0);
        double[] instanceValues = new double[trainingData.numAttributes()];
        System.arraycopy(features, 0, instanceValues, 0, features.length);
        predictData.add(new weka.core.DenseInstance(1.0, instanceValues));
        predictData.setClassIndex(trainingData.numAttributes() - 1);
        return predictData;
    }

}
