package org.urbcomp.cupid.db.util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.Candidate;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.ProgramSolver;

import java.util.List;

public class SolverUtils {

    public static double maximizeScore(List<Candidate> candidates, double punishFactor1, double punishFactor2) {
        int size = candidates.size();
        punishFactor1 *= 10.0;
        punishFactor2 = Math.exp(-punishFactor2 * size);

        int featureNum = 2;
        double[][] scoreMatrix = new double[featureNum][candidates.size()];
        for (int i = 0; i < candidates.size(); ++i) {
            scoreMatrix[0][i] = candidates.get(i).position_score;
            scoreMatrix[1][i] = candidates.get(i).velocity_score;
        }
        RealVector result = ProgramSolver.getResult(scoreMatrix, size, featureNum, punishFactor1, punishFactor2);
        double[] resultArray = result.toArray();

        RealVector weight = result.getSubVector(0, featureNum);
        RealMatrix score = MatrixUtils.createRealMatrix(scoreMatrix);
        double totalScore = 0.0;
        for (int i = 0; i < size; ++i) {
            candidates.get(i).probability = resultArray[featureNum + i];
            totalScore += weight.dotProduct(score.getColumnVector(i)) * resultArray[featureNum + i];
        }
        return totalScore;
    }
}
