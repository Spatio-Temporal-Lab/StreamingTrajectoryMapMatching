package org.urbcomp.cupid.db.algorithm.predict2;

import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import weka.core.Attribute;
import weka.core.DenseInstance;
import weka.core.Instances;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Objects;

public class DataPreparation {

    private static double minLat = Double.MAX_VALUE;
    private static double maxLat = Double.MIN_VALUE;
    private static double minLng = Double.MAX_VALUE;
    private static double maxLng = Double.MIN_VALUE;
    private static final int trajectoriesCount = 2000;

    public static Instances createTrainingData(String observationsPath) {
        List<Trajectory> trajectories = new ArrayList<>();

        // 第一遍遍历以获取最大最小值
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

        // 创建Weka Instances
        ArrayList<Attribute> attributes = new ArrayList<>();
        attributes.add(new Attribute("lat"));
        attributes.add(new Attribute("lng"));
        attributes.add(new Attribute("hour"));
        attributes.add(new Attribute("minute"));
        attributes.add(new Attribute("second"));
        attributes.add(new Attribute("targetLat"));
        attributes.add(new Attribute("targetLng"));

        Instances dataSet = new Instances("TrajectoryData", attributes, 0);

        for (Trajectory observationTrajectory : trajectories) {
            List<GPSPoint> observationPointList = observationTrajectory.getGPSPointList();

            for (int i = 0; i < observationPointList.size() - 1; i++) {
                GPSPoint currentPoint = observationPointList.get(i);
                GPSPoint nextPoint = observationPointList.get(i + 1);

                double[] instanceValues = new double[7];
                instanceValues[0] = normalizeLat(currentPoint.getLat());
                instanceValues[1] = normalizeLng(currentPoint.getLng());

                Calendar cal = Calendar.getInstance();
                cal.setTime(currentPoint.getTime());
                instanceValues[2] = cal.get(Calendar.HOUR_OF_DAY);
                instanceValues[3] = cal.get(Calendar.MINUTE);
                instanceValues[4] = cal.get(Calendar.SECOND);

                instanceValues[5] = normalizeLat(nextPoint.getLat());
                instanceValues[6] = normalizeLng(nextPoint.getLng());

                dataSet.add(new DenseInstance(1.0, instanceValues));
            }
        }

        dataSet.setClassIndex(dataSet.numAttributes() - 1); // 最后一个属性作为目标变量
        return dataSet;
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
