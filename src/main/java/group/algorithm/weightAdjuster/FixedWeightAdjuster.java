package group.algorithm.weightAdjuster;

public class FixedWeightAdjuster extends WeightAdjuster {

    public FixedWeightAdjuster(){}

    public FixedWeightAdjuster(double initialTransitionWeight, double initialEmissionWeight) {
        super(initialTransitionWeight, initialEmissionWeight);
    }
}