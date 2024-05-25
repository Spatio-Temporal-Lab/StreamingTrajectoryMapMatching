package org.urbcomp.cupid.db.algorithm.predict;

import org.deeplearning4j.datasets.iterator.utilty.ListDataSetIterator;
import org.nd4j.linalg.dataset.DataSet;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.factory.Nd4j;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

public class DataPreparation {

    private static double minLat = Double.MAX_VALUE;
    private static double maxLat = Double.MIN_VALUE;
    private static double minLng = Double.MAX_VALUE;
    private static double maxLng = Double.MIN_VALUE;
    private static final int trajectoriesCount = 20;

    public static DataSetIterator createTrainingData(String observationsPath, int sequenceLength) {
        List<Trajectory> trajectories = new ArrayList<>();

        try (InputStream in = ModelGenerator.class.getClassLoader().getResourceAsStream(observationsPath);
             BufferedReader br = new BufferedReader(new InputStreamReader(Objects.requireNonNull(in)))) {

            String trajStr;
            int count = 0;
            while ((trajStr = br.readLine()) != null && count < trajectoriesCount) {
                count++;
                Trajectory observationTrajectory = ModelGenerator.generateTrajectoryByStr(trajStr, -1);
                trajectories.add(observationTrajectory);
                updateMinMax(observationTrajectory.getGPSPointList());
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        List<DataSet> dataSets = new ArrayList<>();
        ArrayList<double[][]> allIn = new ArrayList<>();
        ArrayList<double[]> allOut = new ArrayList<>();
        for (Trajectory observationTrajectory : trajectories) {
            List<GPSPoint> observationPointList = observationTrajectory.getGPSPointList();
            int size = observationPointList.size();
            if (size > 2 * sequenceLength) {
                int numTrain = size - 2 * sequenceLength;
                double[][] inputFeatures = new double[2][sequenceLength];
                double[] outputFeatures = new double[2];
                for (int i = 0; i < numTrain; i++) {
                    for (int j = 0; j < sequenceLength; j++) {
                        inputFeatures[0][j] = normalizeLat(observationPointList.get(i + j).getLat());
                        inputFeatures[1][j] = normalizeLng(observationPointList.get(i + j).getLng());

                    }
                    outputFeatures[0] = normalizeLat(observationPointList.get(i + sequenceLength +1).getLat());
                    outputFeatures[1] = normalizeLng(observationPointList.get(i + sequenceLength +1).getLng());
                    allIn.add(inputFeatures);
                    allOut.add(outputFeatures);
                }
            }
        }
        double[][][] concatenatedInputFeatures = new double[allIn.size()][2][sequenceLength];
        double[][] concatenatedOutputFeatures = new double[allIn.size()][2];
        for (int i = 0; i < allIn.size(); i++) {
            concatenatedInputFeatures[i] = allIn.get(i);
        }for (int i = 0; i < allIn.size(); i++) {
            concatenatedOutputFeatures[i] = allOut.get(i);
        }
        dataSets.add(new DataSet(Nd4j.create(concatenatedInputFeatures), Nd4j.create(concatenatedOutputFeatures)));
        return new ListDataSetIterator<>(dataSets, 64);
    }

    private static void updateMinMax(List<GPSPoint> points) {
        for (GPSPoint point : points) {
            if (point.getLat() < minLat) {
                minLat = point.getLat();
            }
            if (point.getLat() > maxLat) {
                maxLat = point.getLat();
            }
            if (point.getLng() < minLng) {
                minLng = point.getLng();
            }
            if (point.getLng() > maxLng) {
                maxLng = point.getLng();
            }
        }
    }

    private static double normalizeLat(double lat) {
        return (lat - minLat) / (maxLat - minLat);
    }

    private static double normalizeLng(double lng) {
        return (lng - minLng) / (maxLng - minLng);
    }

    public static double denormalizeLat(double normalizedLat) {
        return normalizedLat * (maxLat - minLat) + minLat;
    }

    public static double denormalizeLng(double normalizedLng) {
        return normalizedLng * (maxLng - minLng) + minLng;
    }
}