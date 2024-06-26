package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;


public class CandidateScores {
    public double positionScore;
    public double velocityScore;

    public CandidateScores(double positionScore, double velocityScore) {
        this.positionScore = positionScore;
        this.velocityScore = velocityScore;
    }

    public static double calculatePositionScore(double deltaPos, double posSigma) {
        double fraction = Math.pow(deltaPos / posSigma, 2.0);
        return Math.sqrt(2.0) * Math.exp(-fraction / 2.0);
    }

    public static double calculateVelocityScore(double velocity, double v0, double deltaV) {
        double vMax = v0 + deltaV;
        double c = Math.pow(v0 + vMax, 2.0) * 3.0 / (v0 * 8.0 + vMax * 4.0);
        if (velocity < v0) return c * 2.0 / (v0 + vMax);
        else if (velocity < vMax) return c * 2.0 * (vMax - velocity) / (Math.pow(vMax, 2.0) - Math.pow(v0, 2.0));
        else return 0.0;
    }
}
