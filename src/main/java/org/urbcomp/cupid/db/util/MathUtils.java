package org.urbcomp.cupid.db.util;

import java.util.Collections;
import java.util.List;

public class MathUtils {
    private static double calculateMean(List<Double> values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }
    private static double calculateStdDev(List<Double> values, double mean) {
        double sum = 0.0;
        for (double value : values) {
            sum += Math.pow(value - mean, 2);
        }
        return Math.sqrt(sum / values.size());
    }
    public static double calculateMeanMinus3Std(List<Double> values) {
        // 计算平均值
        double mean = calculateMean(values);

        // 计算标准差
        double std = calculateStdDev(values, mean);

        // 返回平均值减去3倍标准差
        return mean - 3 * std;
    }
    public static double calculate5thPercentile(List<Double> errors) {
        // 先对列表进行排序
        Collections.sort(errors);

        // 计算5th percentile的位置
        int index = (int) Math.ceil(0.05 * errors.size()) - 1;

        // 返回5th percentile的值
        return errors.get(index);
    }
    public static double calculateCorrelation(List<Double> x, List<Double> y) {
        if (x.size() != y.size()) {
            throw new IllegalArgumentException("Lists must have the same size");
        }
        int n = x.size();
        // 计算平均值
        double meanX = calculateMean(x);
        double meanY = calculateMean(y);
        // 计算协方差
        double covariance = 0.0;
        for (int i = 0; i < n; i++) {
            covariance += (x.get(i) - meanX) * (y.get(i) - meanY);
        }
        covariance /= n;
        // 计算标准差
        double stdDevX = calculateStdDev(x, meanX);
        double stdDevY = calculateStdDev(y, meanY);
        // 计算协相关系数
        return covariance / (stdDevX * stdDevY);
    }

    public static int signFun(double x) {
        if (x > 0) return 1;
        else return 0;
    }
}
