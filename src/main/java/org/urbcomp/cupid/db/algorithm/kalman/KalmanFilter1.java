package org.urbcomp.cupid.db.algorithm.kalman;

import org.apache.commons.math3.linear.*;

import java.util.Date;

public class KalmanFilter1 {
    private RealMatrix F; // 状态转移矩阵
    private RealMatrix H; // 观测矩阵
    private RealMatrix Q; // 过程噪声协方差矩阵
    private RealMatrix R; // 观测噪声协方差矩阵
    private RealMatrix P; // 估计误差协方差矩阵
    private RealMatrix B; // 控制矩阵
    private RealMatrix X; // 状态向量
    private RealMatrix Z; // 观测向量
    private RealMatrix pre_X; // 上一次的状态向量
    private RealMatrix obX; // 观测状态向量
    private RealMatrix pre_ObX; // 上一次的观测状态向量
    private double speed_x; // x方向速度
    private double speed_y; // y方向速度
    private int state; // 滤波器状态
    private int n; // 状态向量的维度
    private Date time_stamp; // 时间戳
    private double duration; // 持续时间

    public KalmanFilter1(double[][] F, double[][] H, double gps_var, double pre_var) {
        // 初始化卡尔曼滤波器
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
        // 设置初始状态
        this.X = MatrixUtils.createRealMatrix(this.n, 1);
        this.obX = MatrixUtils.createRealMatrix(this.n, 1);
        this.obX.setEntry(0, 0, x);
        this.obX.setEntry(1, 0, y);
        this.pre_ObX = obX.copy();
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
        // 处理新观测数据
        if (this.state == 0) {
            this.setState(x, y, time_stamp);
            this.state = 1;
            return new double[]{x, y};
        }

        this.duration = (time_stamp.getTime() - this.time_stamp.getTime()) / 1000.0;
        this.time_stamp = time_stamp;

        this.obX.setEntry(0, 0, x);
        this.obX.setEntry(1, 0, y);
        this.F.setEntry(0, 2, this.duration);
        this.F.setEntry(1, 3, this.duration);
        this.speed_x = (this.obX.getEntry(0, 0) - this.pre_ObX.getEntry(0, 0)) / this.duration;
        this.speed_y = (this.obX.getEntry(1, 0) - this.pre_ObX.getEntry(1, 0)) / this.duration;
        this.pre_ObX = obX.copy();

        this.predict();
//        this.update(x, y, time_stamp);
        return new double[]{this.X.getEntry(0, 0), this.X.getEntry(1, 0)};
    }

    private void predict() {
        // 预测下一状态
        this.X = this.F.multiply(this.X).add(this.B);
        this.P = this.F.multiply(this.P).multiply(this.F.transpose()).add(this.Q);
    }

    public void update(double xx, double yy, Date time_stamp) {
        if (this.Z == null){
            this.setState(xx, yy, time_stamp);
            this.state = 1;
            return;
        }
        this.Z = MatrixUtils.createColumnRealMatrix(new double[]{xx, yy, this.speed_x, this.speed_y});
        RealMatrix y = this.Z.subtract(this.H.multiply(this.X));
        RealMatrix S = this.R.add(this.H.multiply(this.P).multiply(this.H.transpose()));
        RealMatrix K = this.P.multiply(this.H.transpose()).multiply(new LUDecomposition(S).getSolver().getInverse());
        this.X = this.X.add(K.multiply(y));
        this.pre_X = this.X.copy();
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(this.n);
        this.P = I.subtract(K.multiply(this.H)).multiply(this.P);
    }

    public void setReal(double xx, double yy){
        this.X.setEntry(0,0, xx);
        this.X.setEntry(1,0, yy);
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
