package org.urbcomp.cupid.db.algorithm.kalman;

import org.apache.commons.math3.linear.*;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.Date;

public class AdaptiveKalmanFilter extends KalmanFilter {

    private double filteredPointX = 0;
    private double filteredPointY = 0;


    public AdaptiveKalmanFilter(double[][] F, double[][] H, double gps_var, double pre_var) {
        super(F, H, gps_var, pre_var);
    }

    public double[] process(double x, double y, Date time_stamp) {
        double[] filteredPoint = super.process(x, y, time_stamp);
        filteredPointX = filteredPoint[0];
        filteredPointY = filteredPoint[1];
        return filteredPoint;
    }


    public void updateNoiseCovariances(double trueX, double trueY) {

        double errorX = GeoFunctions.getDistanceInM(trueX, 0, filteredPointX, 0);
        double errorY = GeoFunctions.getDistanceInM(0, trueY, 0, filteredPointY);
        System.out.println("errorX: " + errorX + " errorY: " + errorY);

        RealMatrix R = getR();
        RealMatrix Q = getQ();

        double alpha = 0.5; // 学习率

        // 根据误差调整观测噪声协方差矩阵 R
        R.setEntry(0, 0, (1 - alpha) * R.getEntry(0, 0) + alpha * errorX * errorX);
        R.setEntry(1, 1, (1 - alpha) * R.getEntry(1, 1) + alpha * errorY * errorY);

        // 根据误差调整过程噪声协方差矩阵 Q
        Q.setEntry(2, 2, (1 - alpha) * Q.getEntry(2, 2) + alpha * errorX * errorX);
        Q.setEntry(3, 3, (1 - alpha) * Q.getEntry(3, 3) + alpha * errorY * errorY);

        setR(R);
        setQ(Q);
    }
}
