package org.urbcomp.cupid.db.algorithm.weightAdjuster;

public class FixedWeightAdjuster extends WeightAdjuster {

    public FixedWeightAdjuster(){}

    public FixedWeightAdjuster(double initialTransitionWeight, double initialEmissionWeight) {
        super(initialTransitionWeight, initialEmissionWeight);
    }

    @Override
    public void updateWeights(double emissionLogProb, double transitionLogProb) {}
}
