package org.urbcomp.cupid.db.algorithm.kalman;

import org.apache.commons.math3.linear.*;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.io.*;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class KalmanFilter {
    private RealMatrix F;
    private RealMatrix H;
    private RealMatrix Q;
    private RealMatrix R;
    private RealMatrix P;
    private RealMatrix B;
    private RealMatrix X;
    private RealMatrix Z;
    private RealMatrix pre_X;
    private double speed_x;
    private double speed_y;
    private int state;
    private int n;
    private Date time_stamp;
    private double duration;

    public KalmanFilter(double[][] F, double[][] H, double gps_var, double pre_var) {
        this.F = MatrixUtils.createRealMatrix(F);
        this.H = MatrixUtils.createRealMatrix(H);
        this.n = F.length;
        this.Q = MatrixUtils.createRealMatrix(this.n, this.n);
        this.Q.setEntry(2, 2, pre_var);
        this.Q.setEntry(3, 3, pre_var);
        this.R = MatrixUtils.createRealMatrix(this.n, this.n);
        this.R.setEntry(0, 0, gps_var);
        this.R.setEntry(1, 1, gps_var);
        this.R.setEntry(2, 2, gps_var);
        this.R.setEntry(3, 3, gps_var);
        this.P = MatrixUtils.createRealIdentityMatrix(this.n);
        this.B = MatrixUtils.createRealMatrix(this.n, 1);
        this.state = 0;
    }

    public void setState(double x, double y, Date time_stamp) {
        this.X = MatrixUtils.createRealMatrix(this.n, 1);
        this.speed_x = 0;
        this.speed_y = 0;
        this.X.setEntry(0, 0, x);
        this.X.setEntry(1, 0, y);
        this.X.setEntry(2, 0, this.speed_x);
        this.X.setEntry(3, 0, this.speed_y);
        this.pre_X = this.X.copy();
        this.time_stamp = time_stamp;
        this.duration = 0;
    }

    public double[] process(double x, double y, Date time_stamp) {
        if (this.state == 0) {
            this.setState(x, y, time_stamp);
            this.state = 1;
            return new double[]{x, y};
        }

        this.duration = (time_stamp.getTime() - this.time_stamp.getTime()) / 1000.0;
        this.time_stamp = time_stamp;

        this.F.setEntry(0, 2, this.duration);
        this.F.setEntry(1, 3, this.duration);
        this.predict();
//        this.update();
        return new double[]{this.X.getEntry(0, 0), this.X.getEntry(1, 0)};
    }

    private void predict() {
        this.X = this.F.multiply(this.X).add(this.B);
        this.P = this.F.multiply(this.P).multiply(this.F.transpose()).add(this.Q);
    }

    public void update(double xx, double yy, Date time_stamp) {
        if (this.Z == null){
            this.setState(xx, yy, time_stamp);
            this.Z = MatrixUtils.createColumnRealMatrix(new double[]{xx, yy, this.speed_x, this.speed_y});
            return;
        }
        this.Z = MatrixUtils.createColumnRealMatrix(new double[]{xx, yy, this.speed_x, this.speed_y});
        RealMatrix y = this.Z.subtract(this.H.multiply(this.X));
        RealMatrix S = this.R.add(this.H.multiply(this.P).multiply(this.H.transpose()));
        RealMatrix K = this.P.multiply(this.H.transpose()).multiply(new LUDecomposition(S).getSolver().getInverse());
        this.X = this.X.add(K.multiply(y));
        this.speed_x = (this.X.getEntry(0, 0) - this.pre_X.getEntry(0, 0)) / this.duration;
        this.speed_y = (this.X.getEntry(1, 0) - this.pre_X.getEntry(1, 0)) / this.duration;
        this.pre_X = this.X.copy();
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(this.n);
        this.P = I.subtract(K.multiply(this.H)).multiply(this.P);
    }
    public RealMatrix getR() {
        return this.R;
    }

    public void setR(RealMatrix R) {
        this.R = R;
    }

    public RealMatrix getQ() {
        return this.Q;
    }

    public void setQ(RealMatrix Q) {
        this.Q = Q;
    }

}
