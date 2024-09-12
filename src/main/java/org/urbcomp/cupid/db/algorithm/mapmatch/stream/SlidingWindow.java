package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

/**
 * 滑动窗口，可以计算窗口内误差均值和标准差
 */
public class SlidingWindow {
    private int windowSize;

    // 三种误差的滑动窗口
    private double[] positionWindow;
    private double[] directionWindow;
    private double[] speedWindow;

    private int index;
    private int count;

    // 三种误差的和
    private double positionSum;
    private double directionSum;
    private double speedSum;

    // 三种误差的平方和
    private double positionSumOfSquares;
    private double directionSumOfSquares;
    private double speedSumOfSquares;

    public SlidingWindow(int size) {
        this.windowSize = size;

        this.positionWindow = new double[size];
        this.directionWindow = new double[size];
        this.speedWindow = new double[size];

        this.index = 0;
        this.count = 0;

        this.positionSum = 0.0;
        this.directionSum = 0.0;
        this.speedSum = 0.0;

        this.positionSumOfSquares = 0.0;
        this.directionSumOfSquares = 0.0;
        this.speedSumOfSquares = 0.0;
    }

    public void addError(double positionError, double directionError, double speedError) {
        if (count < windowSize) {
            // 初始阶段，窗口未满
            positionWindow[index] = positionError;
            directionWindow[index] = directionError;
            speedWindow[index] = speedError;

            positionSum += positionError;
            directionSum += directionError;
            speedSum += speedError;

            positionSumOfSquares += positionError * positionError;
            directionSumOfSquares += directionError * directionError;
            speedSumOfSquares += speedError * speedError;

            count++;
        } else {
            // 窗口已满，替换旧值
            double oldPositionError = positionWindow[index];
            double oldDirectionError = directionWindow[index];
            double oldSpeedError = speedWindow[index];

            positionWindow[index] = positionError;
            directionWindow[index] = directionError;
            speedWindow[index] = speedError;

            positionSum = positionSum - oldPositionError + positionError;
            directionSum = directionSum - oldDirectionError + directionError;
            speedSum = speedSum - oldSpeedError + speedError;

            positionSumOfSquares = positionSumOfSquares - oldPositionError * oldPositionError + positionError * positionError;
            directionSumOfSquares = directionSumOfSquares - oldDirectionError * oldDirectionError + directionError * directionError;
            speedSumOfSquares = speedSumOfSquares - oldSpeedError * oldSpeedError + speedError * speedError;
        }

        // 移动索引
        index = (index + 1) % windowSize;
    }

    // 获取位置误差的均值
    public double getPositionMean() {
        return positionSum / count;
    }

    // 获取方向误差的均值
    public double getDirectionMean() {
        return directionSum / count;
    }

    // 获取速度误差的均值
    public double getSpeedMean() {
        return speedSum / count;
    }

    // 获取位置误差的标准差
    public double getPositionStandardDeviation() {
        double mean = getPositionMean();
        return Math.sqrt(positionSumOfSquares / count - mean * mean);
    }

    // 获取方向误差的标准差
    public double getDirectionStandardDeviation() {
        double mean = getDirectionMean();
        return Math.sqrt(directionSumOfSquares / count - mean * mean);
    }

    // 获取速度误差的标准差
    public double getSpeedStandardDeviation() {
        double mean = getSpeedMean();
        return Math.sqrt(speedSumOfSquares / count - mean * mean);
    }

    public boolean isFull() {
        return count >= windowSize;
    }
}

