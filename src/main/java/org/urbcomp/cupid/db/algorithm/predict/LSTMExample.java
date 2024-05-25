package org.urbcomp.cupid.db.algorithm.predict;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.deeplearning4j.nn.conf.layers.LSTM;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.optimize.listeners.ScoreIterationListener;
import org.deeplearning4j.nn.weights.WeightInit;
import org.deeplearning4j.nn.conf.Updater;
import org.deeplearning4j.nn.conf.layers.RnnOutputLayer;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.dataset.DataSet;
import org.nd4j.linalg.dataset.api.preprocessor.NormalizerMinMaxScaler;
import org.nd4j.linalg.dataset.api.preprocessor.DataNormalization;
import org.nd4j.linalg.indexing.NDArrayIndex;
import org.nd4j.linalg.learning.config.Adam;
import org.nd4j.linalg.lossfunctions.LossFunctions;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.buffer.DataType;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.ArrayList;

public class LSTMExample {

    public static void main(String[] args) throws IOException {
        int trainNum = 6;
        int perNum = 1;
        boolean setRange = true;

        // 读入数据
        List<String> lines = Files.readAllLines(Paths.get("D:\\StreamingTrajectoryMapMatching\\src\\main\\resources\\data\\20090415134400.txt"));
        List<double[]> dataList = new ArrayList<>();
        for (String line : lines) {
            String[] tokens = line.split(",");
            double[] dataPoint = {Double.parseDouble(tokens[0]), Double.parseDouble(tokens[1])};
            dataList.add(dataPoint);
        }

        double[][] dataArray = new double[dataList.size()][2];
        for (int i = 0; i < dataList.size(); i++) {
            dataArray[i] = dataList.get(i);
        }

        // 归一化
        NormalizerMinMaxScaler scaler = new NormalizerMinMaxScaler(0, 1);
        INDArray data = Nd4j.create(dataArray);

//        scaler.fit(data);
//        scaler.transform(data);

        // 生成训练数据
        INDArray[] dataset = createDataset(data, trainNum, perNum);
        INDArray trainX = dataset[0];
        INDArray trainY = dataset[1];
        INDArray testX = dataset[2];
        INDArray testY = dataset[3];

        // 训练模型
        MultiLayerNetwork model = trainModel(trainX, trainY);

        // 保存模型
        model.save(new File("traj_model_120.zip"), true);
    }

    public static INDArray[] createDataset(INDArray data, int nPredictions, int nNext) {
        int dim = (int) data.size(1);
        List<INDArray> trainXList = new ArrayList<>();
        List<INDArray> trainYList = new ArrayList<>();
        for (int i = 0; i < data.size(0) - nPredictions - nNext - 1; i++) {
            INDArray a = data.get(NDArrayIndex.interval(i, i + nPredictions), NDArrayIndex.all());
            trainXList.add(a);
            INDArray tempb = data.get(NDArrayIndex.interval(i + nPredictions, i + nPredictions + nNext), NDArrayIndex.all());
            INDArray b = Nd4j.toFlattened(tempb);
            trainYList.add(b);
        }

        INDArray trainX = Nd4j.create(trainXList, new long[]{trainXList.size(), nPredictions, dim});
        INDArray trainY = Nd4j.create(trainYList, new long[]{trainYList.size(), nNext * dim});

        INDArray testX = data.get(NDArrayIndex.interval(data.size(0) - nPredictions - nNext - 1, data.size(0) - nNext - 1), NDArrayIndex.all());
        INDArray tempb = data.get(NDArrayIndex.interval(data.size(0) - nPredictions - nNext - 1 + nPredictions, data.size(0) - 1), NDArrayIndex.all());
        INDArray testY = Nd4j.toFlattened(tempb);

        return new INDArray[]{trainX, trainY, testX.reshape(1, nPredictions, dim), testY.reshape(1, nNext * dim)};
    }

    public static MultiLayerNetwork trainModel(INDArray trainX, INDArray trainY) {
        int lstmLayerSize = 120;
        int nOut = (int) trainY.size(1);

        NeuralNetConfiguration.ListBuilder conf = new NeuralNetConfiguration.Builder()
                .seed(123)
                .weightInit(WeightInit.XAVIER)
                .updater(new Adam(0.001))
                .list()
                .layer(0, new LSTM.Builder()
                        .nIn(trainX.size(2))
                        .nOut(lstmLayerSize)
                        .activation(Activation.TANH)
                        .build())
                .layer(1, new LSTM.Builder()
                        .nIn(lstmLayerSize)
                        .nOut(lstmLayerSize)
                        .activation(Activation.TANH)
                        .build())
                .layer(2, new DenseLayer.Builder()
                        .nIn(lstmLayerSize)
                        .nOut(nOut)
                        .activation(Activation.RELU)
                        .build())
                .layer(3, new RnnOutputLayer.Builder(LossFunctions.LossFunction.MSE)
                        .activation(Activation.IDENTITY)
                        .nIn(nOut)
                        .nOut(nOut)
                        .build());

        MultiLayerNetwork model = new MultiLayerNetwork(conf.build());
        model.init();
        model.setListeners(new ScoreIterationListener(10));

        DataSet trainData = new DataSet(trainX, trainY);
        for (int i = 0; i < 100; i++) {
            model.fit(trainData);
        }

        return model;
    }
}
