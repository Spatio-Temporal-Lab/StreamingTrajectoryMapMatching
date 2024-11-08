package group.algorithm.weightAdjuster;

public abstract class WeightAdjuster {
    protected double transitionWeight = 0.5;
    protected double emissionWeight = 0.5;
    protected double learningRate = 0.01;

    public WeightAdjuster() {}

    public WeightAdjuster(double initialTransitionWeight, double initialEmissionWeight) {
        this.transitionWeight = initialTransitionWeight;
        this.emissionWeight = initialEmissionWeight;
    }

    public WeightAdjuster(double initialTransitionWeight, double initialEmissionWeight, double learningRate) {
        this.transitionWeight = initialTransitionWeight;
        this.emissionWeight = initialEmissionWeight;
        this.learningRate = learningRate;
    }

    public void updateWeights(double emissionLogProb, double transitionLogProb) {}

    public double getTransitionWeight() {
        return transitionWeight;
    }

    public double getEmissionWeight() {
        return emissionWeight;
    }
}
