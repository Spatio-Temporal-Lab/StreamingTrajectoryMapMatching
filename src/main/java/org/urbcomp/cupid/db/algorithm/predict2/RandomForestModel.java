package org.urbcomp.cupid.db.algorithm.predict2;

import weka.classifiers.trees.RandomForest;
import weka.core.Instances;
import weka.core.SerializationHelper;

public class RandomForestModel {

    public static void main(String[] args) throws Exception {
        String observationsPath = "data/output.txt";
        Instances trainingData = DataPreparation.createTrainingData(observationsPath);

        // 设置标签属性的索引
        trainingData.setClassIndex(trainingData.numAttributes() - 1);

        // 创建并训练随机森林模型
        RandomForest randomForest = new RandomForest();
        randomForest.buildClassifier(trainingData);

        // 保存模型
        SerializationHelper.write("randomForestModel.model", randomForest);


    }


}
