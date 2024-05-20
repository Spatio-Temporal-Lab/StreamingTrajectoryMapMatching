package org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi;

import static org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi.Auxiliary.*;

public class StandardViterbiDecoder {
    private final int K;
    private final int T;
    private final double[][] scores;
    private final int[][] path;
    private final int[] optimalPath;

    public StandardViterbiDecoder(int K, int T) {
        this.K = K;
        this.T = T;
        this.scores = new double[K][T];
        this.path = new int[K][T];
        this.optimalPath = new int[T];
    }

    public void initialization(int[] observations, double[] initial, double[][] A, double[][] E) {
        for (int j = 0; j < K; j++) {
            double max_val = LOWER_BOUND;
            int max_index = 0;
            for (int i = 0; i < K; i++) {
                double aux = boundedLogSum(boundedLog(initial[i]), boundedLog(A[i][j]), boundedLog(E[j][observations[0]]));
                if (aux > max_val) {
                    max_val = aux;
                    max_index = i;
                }
            }
            scores[j][0] = max_val;
            path[j][0] = max_index;
        }
    }

    public void recursion(int[] observations, double[][] A, double[][] E) {
        for (int t = 1; t < T; t++) {
            for (int j = 0; j < K; j++) {
                double max_val = LOWER_BOUND;
                int max_index = 0;
                for (int i = 0; i < K; i++) {
                    double aux = boundedLogSum(scores[i][t - 1], boundedLog(A[i][j]), boundedLog(E[j][observations[t]]));
                    if (aux > max_val) {
                        max_val = aux;
                        max_index = i;
                    }
                }
                scores[j][t] = max_val;
                path[j][t] = max_index;
            }
        }
    }

    public void termination() {
        double max_val = LOWER_BOUND;
        int max_index = 0;
        for (int j = 0; j < K; j++) {
            if (scores[j][T - 1] > max_val) {
                max_val = scores[j][T - 1];
                max_index = j;
            }
        }
        optimalPath[T - 1] = max_index;
        for (int t = T - 2; t >= 0; t--) {
            optimalPath[t] = path[optimalPath[t + 1]][t + 1];
        }
    }

    public void viterbi(int[] observations, double[] initial, double[][] A, double[][] E) {
        initialization(observations, initial, A, E);
        recursion(observations, A, E);
        termination();
    }

    public int[] getOptimalPath() {
        return this.optimalPath;
    }
}