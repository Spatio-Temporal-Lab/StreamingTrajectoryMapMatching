package org.urbcomp.cupid.db.algorithm.mapmatch.stream;

public class DynamicWeightOptimizer {
    private double transitionWeight = 0.5;
    private double emissionWeight = 0.5;
    private double learningRate = 0.01;

    public DynamicWeightOptimizer() {}

    public DynamicWeightOptimizer(double initialTransitionWeight, double initialEmissionWeight, double learningRate) {
        this.transitionWeight = initialTransitionWeight;
        this.emissionWeight = initialEmissionWeight;
        this.learningRate = learningRate;
    }

    public void updateWeights(double emissionLogProb, double transitionLogProb) {
        double gradEmission = Math.abs(emissionLogProb);
        double gradTransition = Math.abs(transitionLogProb);

        if (gradTransition > 30) {
            gradTransition = 0;
        }
        // 更新权重
        transitionWeight += learningRate * gradTransition;
        emissionWeight += learningRate * gradEmission;

        // 归一化权重，确保 w_t + w_e = 1
        double sum = transitionWeight + emissionWeight;
        if (sum != 0) {
            transitionWeight /= sum;
            emissionWeight /= sum;
        } else {
            // 防止除以零
            transitionWeight = 0.5;
            emissionWeight = 0.5;
        }
    }

    public double getTransitionWeight() {
        return transitionWeight;
    }

    public double getEmissionWeight() {
        return emissionWeight;
    }
}
