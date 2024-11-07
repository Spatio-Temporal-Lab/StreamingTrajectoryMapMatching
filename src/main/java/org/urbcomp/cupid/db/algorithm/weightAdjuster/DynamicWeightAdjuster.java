package org.urbcomp.cupid.db.algorithm.weightAdjuster;

public class DynamicWeightAdjuster extends WeightAdjuster{

    public DynamicWeightAdjuster() {}

    public DynamicWeightAdjuster(double initialTransitionWeight, double initialEmissionWeight) {
        this.transitionWeight = initialTransitionWeight;
        this.emissionWeight = initialEmissionWeight;
    }

    public void updateWeights(double emissionLogProb, double transitionLogProb) {
        double gradEmission = Math.abs(emissionLogProb);
        double gradTransition = Math.abs(transitionLogProb);

        if (gradTransition > 30) {
            gradTransition = 0;
        }

        transitionWeight += learningRate * gradTransition;
        emissionWeight += learningRate * gradEmission;

        double sum = transitionWeight + emissionWeight;
        if (sum != 0) {
            transitionWeight /= sum;
            emissionWeight /= sum;
        } else {

            transitionWeight = 0.5;
            emissionWeight = 0.5;
        }
    }
}
