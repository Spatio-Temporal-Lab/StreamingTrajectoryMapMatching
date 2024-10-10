package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.*;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ProgramSolver {
    private static int featureNum;
    private int candidateNum;
    private final double lambda1;
    private final double lambda2;

    private RealMatrix eMatrix;
    private RealMatrix hMatrix;
    private RealMatrix aMatrix;

    private ProgramSolver(double lambda1, double lambda2) {
        this.lambda1 = lambda1;
        this.lambda2 = lambda2;
    }

    public static RealVector getResult(double[][] scoreMatrix, int candidateNum, int features, double lambda1, double lambda2) {
        featureNum = features;
        ProgramSolver compute = new ProgramSolver(lambda1, lambda2);
        compute.init(scoreMatrix, candidateNum);
        return compute.start();
    }

    void init(double[][] matrix, int candidateNum) {
        this.candidateNum = candidateNum;

        double[][] eArrays = new double[2][featureNum + candidateNum];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < featureNum + candidateNum; ++j) {
                eArrays[i][j] = ((i == 0 && j < featureNum) || (i == 1 && j >= featureNum)) ? 1 : 0;
            }
        }
        eMatrix = MatrixUtils.createRealMatrix(eArrays);

        double[][] sArrays = new double[matrix.length][matrix[0].length];
        for (int i = 0; i < featureNum; i++) {
            if (candidateNum >= 0) System.arraycopy(matrix[i], 0, sArrays[i], 0, candidateNum);
        }
        RealMatrix s = MatrixUtils.createRealMatrix(sArrays);

        RealMatrix sTranspose = s.transpose();
        double[][] stArrays = sTranspose.getData();

        double[][] hArrays = new double[featureNum + candidateNum][featureNum + candidateNum];
        for (int i = 0; i < featureNum + candidateNum; i++) {
            for (int j = 0; j < featureNum + candidateNum; j++) {
                if (i < featureNum && j < featureNum) {
                    hArrays[i][j] = (i == j) ? lambda1 : 0;
                } else if (j >= featureNum && i < featureNum) {
                    hArrays[i][j] = -0.5 * sArrays[i][j - featureNum];
                } else if (j < featureNum) {
                    hArrays[i][j] = -0.5 * stArrays[i - featureNum][j];
                } else {
                    hArrays[i][j] = (i == j) ? lambda2 : 0;
                }
            }
        }
        hMatrix = MatrixUtils.createRealMatrix(hArrays);

        double[] aArrays = new double[featureNum + candidateNum];
        Arrays.fill(aArrays, 1.0);
        aMatrix = MatrixUtils.createRealDiagonalMatrix(aArrays);
    }

    public RealVector start() {
        double[] xArray = new double[featureNum + candidateNum];
        xArray[featureNum - 1] = 1;
        xArray[featureNum + candidateNum - 1] = 1;
        RealVector x = MatrixUtils.createRealVector(xArray);
        PointValuePair solution = null;

        for (int iteration = 0; iteration < 10; iteration++) {
            List<double[]> effectiveConstraint = new ArrayList<>();
            List<double[]> nonEffectiveConstraint = new ArrayList<>();
            for (int i = 0; i < featureNum + candidateNum; i++) {
                if (Math.abs(x.dotProduct(aMatrix.getRowVector(i))) < 1e-5) {
                    effectiveConstraint.add(aMatrix.getRow(i));
                } else {
                    nonEffectiveConstraint.add(aMatrix.getRow(i));
                }
            }

            double[][] a1Arrays = new double[effectiveConstraint.size()][featureNum + candidateNum];
            double[][] a2Arrays = new double[nonEffectiveConstraint.size()][featureNum + candidateNum];

            for (int i = 0; i < effectiveConstraint.size(); i++) {
                if (featureNum + candidateNum >= 0)
                    System.arraycopy(effectiveConstraint.get(i), 0, a1Arrays[i], 0, featureNum + candidateNum);
            }
            for (int i = 0; i < nonEffectiveConstraint.size(); i++) {
                if (featureNum + candidateNum >= 0)
                    System.arraycopy(nonEffectiveConstraint.get(i), 0, a2Arrays[i], 0, featureNum + candidateNum);
            }

            List<LinearConstraint> constraints = new ArrayList<>();
            if (!effectiveConstraint.isEmpty()) {
                RealMatrix a1 = MatrixUtils.createRealMatrix(a1Arrays);
                for (int i = 0; i < a1.getRowDimension(); i++) {
                    constraints.add(new LinearConstraint(a1.getRow(i), Relationship.GEQ, 0.0));
                }
            }

            RealMatrix a2 = MatrixUtils.createRealMatrix(a2Arrays);
            for (int i = 0; i < 2; i++) {
                constraints.add(new LinearConstraint(eMatrix.getRowVector(i), Relationship.EQ, 0));
            }
            for (int i = 0; i < featureNum + candidateNum; i++) {
                double[] arr = new double[featureNum + candidateNum];
                Arrays.fill(arr, 0.0);
                arr[i] = 1;
                constraints.add(new LinearConstraint(arr, Relationship.GEQ, -1.0));
                constraints.add(new LinearConstraint(arr, Relationship.LEQ, 1.0));
            }

            RealVector xtH = hMatrix.scalarMultiply(2).operate(x);

            LinearObjectiveFunction f = new LinearObjectiveFunction(xtH, 0);
            try {
                solution = new SimplexSolver().optimize(f, new LinearConstraintSet(constraints), GoalType.MINIMIZE);
            } catch (Exception e) {
                e.printStackTrace();
            }

            if (solution != null) {
                double max = solution.getValue();

                if (Math.abs(max) > 1e-5) {
                    double miuMax = 0.0;
                    boolean isAllBiggerThan = true;
                    for (int i = 0; i < a2.getRowDimension(); i++) {
                        if (a2.getRowVector(i).dotProduct(MatrixUtils.createRealVector(solution.getPoint())) < 0.0) {
                            isAllBiggerThan = false;
                            break;
                        }
                    }
                    if (isAllBiggerThan) {
                        miuMax = Double.POSITIVE_INFINITY;
                    } else {
                        double[] b2 = new double[a2.getRowDimension()];
                        double[] toSub = a2.operate(x.toArray());
                        RealVector upper = MatrixUtils.createRealVector(b2).subtract(MatrixUtils.createRealVector(toSub));
                        RealVector a2d = MatrixUtils.createRealVector(a2.operate(solution.getPoint()));
                        RealVector res = upper.ebeDivide(a2d);

                        double minValue = Double.MAX_VALUE;
                        int minInd = -1;
                        for (int i = 0; i < a2d.getDimension(); i++) {
                            if (a2d.getEntry(i) < 0.0) {
                                if (res.getEntry(i) < minValue) {
                                    minValue = res.getEntry(i);
                                    minInd = i;
                                }
                            }
                        }

                        if (minInd != -1) {
                            miuMax = minValue;
                        }
                    }

                    double miuUpper = MatrixUtils.createRealVector(hMatrix.preMultiply(solution.getPoint())).dotProduct(x);
                    double miuLower = MatrixUtils.createRealVector(hMatrix.preMultiply(solution.getPoint())).dotProduct(MatrixUtils.createRealVector(solution.getPoint()));
                    double miu0 = -miuUpper / miuLower;

                    double[] miuList;

                    if (miu0 > miuMax || miu0 < 0) {
                        miuList = new double[]{0.0, miuMax};
                    } else {
                        miuList = new double[]{0.0, miuMax, miu0};
                    }

                    List<Double> value = new ArrayList<>();
                    for (double mu : miuList) {
                        RealVector vector = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(mu));
                        value.add(hMatrix.preMultiply(vector).dotProduct(vector));
                    }

                    int valueMinInd = -1;
                    double valueMinValue = Double.MAX_VALUE;
                    for (int i = 0; i < value.size(); i++) {
                        if (valueMinValue > value.get(i)) {
                            valueMinInd = i;
                            valueMinValue = value.get(i);
                        }
                    }

                    double miu = miuList[valueMinInd];
                    x = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(miu));
                } else {
                    break;
                }
            }
        }
        return x;
    }
}
