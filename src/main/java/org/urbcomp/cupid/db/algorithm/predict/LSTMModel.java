package org.urbcomp.cupid.db.algorithm.predict;

import org.deeplearning4j.nn.api.OptimizationAlgorithm;
import org.deeplearning4j.nn.conf.MultiLayerConfiguration;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.LSTM;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.nn.conf.layers.recurrent.LastTimeStep;
import org.deeplearning4j.nn.conf.preprocessor.RnnToCnnPreProcessor;
import org.deeplearning4j.nn.conf.preprocessor.RnnToFeedForwardPreProcessor;

import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.nn.weights.WeightInit;
import org.deeplearning4j.optimize.listeners.ScoreIterationListener;
import org.deeplearning4j.util.ModelSerializer;
import org.nd4j.evaluation.classification.Evaluation;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.dataset.api.DataSet;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.learning.config.Adam;
import org.nd4j.linalg.lossfunctions.LossFunctions;

import java.io.File;
import java.io.IOException;

public class LSTMModel {
    private MultiLayerNetwork model;

    public LSTMModel(int inputSize, int outputSize, int lstmLayerSize) {
        MultiLayerConfiguration conf = new NeuralNetConfiguration.Builder()
                .seed(123)
                .optimizationAlgo(OptimizationAlgorithm.STOCHASTIC_GRADIENT_DESCENT)
                .updater(new Adam(0.001))
                .list()
                .layer(0, new LSTM.Builder()
                        .nIn(inputSize)
                        .nOut(lstmLayerSize)
                        .activation(Activation.TANH)
                        .build())
                .layer(1, new LastTimeStep(new LSTM.Builder()
                        .nIn(lstmLayerSize)
                        .nOut(lstmLayerSize)
                        .activation(Activation.TANH)
                        .build()))
                .layer(2, new DenseLayer.Builder()
                        .nIn(lstmLayerSize)
                        .nOut(100)
                        .activation(Activation.RELU)
                        .build())
                .layer(3, new OutputLayer.Builder(LossFunctions.LossFunction.MSE)
                        .activation(Activation.IDENTITY)
                        .nIn(100)
                        .nOut(outputSize)
                        .build())
                .build();
        model = new MultiLayerNetwork(conf);
        model.init();
        model.setListeners(new ScoreIterationListener(20));
    }

    public double train(DataSetIterator trainData) {
        trainData.reset();
        model.fit(trainData);
        return model.score();
    }

    public double[][] predict(double[][][] input) {
        INDArray inputNDArray = Nd4j.create(input);
        INDArray output = model.output(inputNDArray, false);
        System.out.println(output);
        return output.toDoubleMatrix(); // 获取最后一个时间步的输出
    }

    public void saveModel(String filePath) throws IOException {
        ModelSerializer.writeModel(model, new File(filePath), true);
    }

    public void loadModel(String filePath) throws IOException {
        model = ModelSerializer.restoreMultiLayerNetwork(new File(filePath));
    }
}