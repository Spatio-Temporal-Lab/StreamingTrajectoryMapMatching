package org.urbcomp.cupid.db.algorithm.predict;


import org.deeplearning4j.datasets.iterator.impl.ListDataSetIterator;
import org.nd4j.linalg.dataset.DataSet;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.factory.Nd4j;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;

import java.io.*;
import java.nio.file.Files;
import java.util.*;

public class DataPreparation {

    private static final String varPath = "var.txt";
    private static double minLat = Double.MAX_VALUE;
    private static double maxLat = Double.MIN_VALUE;
    private static double minLng = Double.MAX_VALUE;
    private static double maxLng = Double.MIN_VALUE;
    private static final int trajectoriesCount = 20000;

    public static DataSetIterator createTrainingData(String observationsPath, int sequenceLength) {
        List<Trajectory> trajectories = new ArrayList<>();

        try (InputStream in = Files.newInputStream(new File(observationsPath).toPath());
             BufferedReader br = new BufferedReader(new InputStreamReader(Objects.requireNonNull(in)))) {
            String trajStr;
            while ((trajStr = br.readLine()) != null) {
                Trajectory observationTrajectory = ModelGenerator.generateTrajectoryByStr(trajStr, -1);
                trajectories.add(observationTrajectory);
                updateMinMax(observationTrajectory.getGPSPointList());
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        Collections.shuffle(trajectories);

        List<Trajectory> subTrajectories =  trajectories.subList(0,trajectoriesCount);
        for (Trajectory tr : subTrajectories){
            updateMinMax(tr.getGPSPointList());
        }
        writeVariablesToFile(varPath);

        List<DataSet> dataSets = new ArrayList<>();
        ArrayList<double[][]> allIn = new ArrayList<>();
        ArrayList<double[]> allOut = new ArrayList<>();
        for (Trajectory observationTrajectory : subTrajectories) {
            List<GPSPoint> observationPointList = observationTrajectory.getGPSPointList();
            int size = observationPointList.size();
            if (size > 2 * sequenceLength) {
                int numTrain = size - 2 * sequenceLength;
                for (int i = 0; i < numTrain; i++) {
                    double[][] inputFeatures = new double[2][sequenceLength];
                    double[] outputFeatures = new double[2];
                    for (int j = 0; j < sequenceLength; j++) {
                        inputFeatures[0][j] = normalizeLat(observationPointList.get(i + j).getLat());
                        inputFeatures[1][j] = normalizeLng(observationPointList.get(i + j).getLng());
                    }
                    outputFeatures[0] = normalizeLat(observationPointList.get(i + sequenceLength).getLat());
                    outputFeatures[1] = normalizeLng(observationPointList.get(i + sequenceLength).getLng());
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
        return new ListDataSetIterator<>(dataSets, 10);
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

    public static void writeVariablesToFile(String filePath) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
            writer.write("minLat: " + minLat);
            writer.newLine();
            writer.write("maxLat: " + maxLat);
            writer.newLine();
            writer.write("minLng: " + minLng);
            writer.newLine();
            writer.write("maxLng: " + maxLng);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void readVariablesFromFile() {
        try (BufferedReader reader = new BufferedReader(new FileReader(varPath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(": ");
                if (parts.length == 2) {
                    switch (parts[0]) {
                        case "minLat":
                            minLat = Double.parseDouble(parts[1]);
                            break;
                        case "maxLat":
                            maxLat = Double.parseDouble(parts[1]);
                            break;
                        case "minLng":
                            minLng = Double.parseDouble(parts[1]);
                            break;
                        case "maxLng":
                            maxLng = Double.parseDouble(parts[1]);
                            break;
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static double normalizeLat(double lat) {
        return (lat - minLat) / (maxLat - minLat);
    }

    public static double normalizeLng(double lng) {
        return (lng - minLng) / (maxLng - minLng);
    }

    public static double denormalizeLat(double normalizedLat) {
        return normalizedLat * (maxLat - minLat) + minLat;
    }

    public static double denormalizeLng(double normalizedLng) {
        return normalizedLng * (maxLng - minLng) + minLng;
    }
}