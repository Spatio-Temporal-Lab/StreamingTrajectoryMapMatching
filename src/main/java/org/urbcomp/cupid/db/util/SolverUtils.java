package org.urbcomp.cupid.db.util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.CandidateAttributes;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.CandidateScores;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.PointCandidateSet;
import org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner.ProgramSolver;
import org.urbcomp.cupid.db.model.point.CandidatePoint;

import java.util.*;

public class SolverUtils {
    /**
     * 计算当前pcSet的总得分，最小路径得分，以及每个候选点的概率、累积概率
     *
     * @param pcSet 当前的候选点和观测点集合
     * @param punishFactor1 得分惩罚系数1
     * @param punishFactor2 得分惩罚系数2
     * @param eta 路径长度惩罚系数
     * @return totalScore 总得分
     */
    public static double maximizeScore(PointCandidateSet pcSet, double punishFactor1, double punishFactor2, double eta) {
        List<CandidatePoint> candidatePoints = pcSet.getCandidatePoints();
        Map<CandidatePoint, CandidateScores> scoresMap = pcSet.scoresMap;
        Map<CandidatePoint, CandidateAttributes> attributesMap = pcSet.getAttributesMap();
//        punishFactor1 *= 10.0;
        punishFactor2 = Math.exp(-punishFactor2 * candidatePoints.size());

        int featureNum = 2;
        double[][] scoreMatrix = new double[featureNum][candidatePoints.size()];
        for (int i = 0; i < candidatePoints.size(); ++i) {
            scoreMatrix[0][i] = scoresMap.get(candidatePoints.get(i)).positionScore;
            scoreMatrix[1][i] = scoresMap.get(candidatePoints.get(i)).velocityScore;
        }

        RealVector result = ProgramSolver.getResult(scoreMatrix, candidatePoints.size(), featureNum, punishFactor1, punishFactor2);
        double[] resultArray = result.toArray();
//        for (double v : resultArray) {
//            System.out.println("result prob: " + v);
//        }
        RealVector weight = result.getSubVector(0, featureNum);
        RealMatrix score = MatrixUtils.createRealMatrix(scoreMatrix);
        double totalScore = 0.0;
        for (int i = 0; i < candidatePoints.size(); ++i) {
            // 计算概率和累积概率
            double accProb = attributesMap.get(candidatePoints.get(i)).accProb;
            attributesMap.get(candidatePoints.get(i)).prob = resultArray[featureNum + i];
            attributesMap.get(candidatePoints.get(i)).accProb = resultArray[featureNum + i] + accProb;
            totalScore += weight.dotProduct(score.getColumnVector(i)) * resultArray[featureNum + i];
        }
        for (CandidatePoint candidatePoint : candidatePoints) {
            // 计算最小路径得分
            CandidateAttributes candidateAttributes = attributesMap.get(candidatePoint);
            double pathScore = candidateAttributes.len - candidateAttributes.accProb * eta;
            if (pathScore < pcSet.minPathScore) pcSet.minPathScore = pathScore;
        }
        return totalScore;
    }
}
