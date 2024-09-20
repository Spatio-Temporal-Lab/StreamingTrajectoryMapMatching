package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

public class SlidingWindow {
    private int windowSize;
    private double[] distances;          // 用于存储距离数据的循环队列
    private double[] transitionMetrics;  // 用于存储转移差值的循环队列
    private int index;                   // 当前插入位置的索引
    private int count;                   // 当前窗口中的数据点个数

    private double distanceSum;          // 距离的累积和
    private double distanceSquaredSum;   // 距离平方的累积和
    private double transitionMetricSum;  // 转移差值的累积和

    private double mu;  // 均值，正态分布的参数
    private double sigma;  // 标准差，正态分布的参数
    private double beta;   // 指数分布的参数

    // 构造函数，初始化循环队列和参数
    public SlidingWindow(int windowSize, double initialSigma, double initialBeta) {
        this.windowSize = windowSize;
        this.distances = new double[windowSize];
        this.transitionMetrics = new double[windowSize];
        this.index = 0;
        this.count = 0;
        this.mu = 0.0;
        this.sigma = initialSigma;
        this.beta = initialBeta;
        this.distanceSum = 0;
        this.distanceSquaredSum = 0;
        this.transitionMetricSum = 0;
    }

    /**
     * 更新滑动窗口，添加新的距离数据并更新 sigma 和 beta
     * @param distance GPS观测点和候选点之间的距离 (用于更新sigma)
     * @param transitionMetric 两个GPS观测点的距离差值 (用于更新beta)
     */
    public void addData(double distance, double transitionMetric) {
        // 如果 count 小于阈值，直接跳过异常检测，正常添加数据
        if (count < 10) {
            addDataToWindow(distance, transitionMetric);
            return;
        }

        // 进行异常值检测
        if (!isAnomalous(distance, distanceSum / count) &&
                !isAnomalous(transitionMetric, transitionMetricSum / count)) {
            // 如果数据不异常，才添加到窗口
            addDataToWindow(distance, transitionMetric);
//        } else {
//            System.out.println("异常数据被检测到并忽略: distance=" + distance + ", transitionMetric=" + transitionMetric);
        }
    }

    // 添加数据到滑动窗口中并更新统计数据
    private void addDataToWindow(double distance, double transitionMetric) {
        // 检查当前窗口是否已满
        if (count == windowSize) {
            // 如果已满，移除最早的数据点贡献
            int removeIndex = index;
            distanceSum -= distances[removeIndex];
            distanceSquaredSum -= Math.pow(distances[removeIndex], 2);
            transitionMetricSum -= transitionMetrics[removeIndex];
        } else {
            // 如果未满，增加窗口中的数据点个数
            count++;
        }

        // 更新循环队列中的值
        distances[index] = distance;
        transitionMetrics[index] = transitionMetric;

        // 更新累积和
        distanceSum += distance;
        distanceSquaredSum += Math.pow(distance, 2);
        transitionMetricSum += transitionMetric;

        // 更新下一个插入位置的索引
        index = (index + 1) % windowSize;

        // 更新 sigma 和 beta
        updateMu();
        updateSigma();
        updateBeta();
    }

    /**
     * 检测某个值是否是异常值，通过 Z-Score 判断
     * @param value 要检测的值
     * @return 如果值是异常值，则返回 true，否则返回 false
     */
    private boolean isAnomalous(double value, double param) {
        return value > 50 * param;
    }

    private void updateMu() {
        if (count >= windowSize) {
            this.mu = distanceSum / count;
        }
    }

    /**
     * 计算并更新sigma (正态分布参数)
     */
    private void updateSigma() {
        if (count >= windowSize) {
            double mean = distanceSum / count;
            double variance = (distanceSquaredSum / count) - Math.pow(mean, 2);
            if (variance > 0) {
                this.sigma = 5 * Math.sqrt(variance);
            }
            if (sigma > 50) {
                sigma = 50;
            }
        }
    }

    /**
     * 计算并更新beta (指数分布参数)
     */
    private void updateBeta() {
        if (count >= windowSize) {
            if (transitionMetricSum / count > 0) {
                this.beta = transitionMetricSum / count;
            }
            if (beta > 5) {
                beta = 5;
            }
        }
    }

    public double getMu() {
        return mu;
    }

    /**
     * 获取当前的sigma值
     * @return sigma (正态分布的标准差)
     */
    public double getSigma() {
        return sigma;
    }

    /**
     * 获取当前的beta值
     * @return beta (指数分布的平均值)
     */
    public double getBeta() {
        return beta;
    }

    public boolean isFull() {
        return count >= windowSize;
    }
}
