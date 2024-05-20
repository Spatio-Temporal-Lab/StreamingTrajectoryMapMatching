package org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi;

import java.util.ArrayList;
import java.util.List;

public class Auxiliary {
    public static final Double LOWER_BOUND = -2e6;
    public static Double boundedLog(Double prob) {
        if (prob == 0) return LOWER_BOUND;
        else return Math.log(prob);
    }
    public static Double boundedLogSum(Double rawProb, Double transProb, Double... args) {
        Double logProbSum = rawProb + transProb;
        for (Double arg: args) {
            logProbSum += arg;
        }
        if (logProbSum < LOWER_BOUND) return LOWER_BOUND;
        else return logProbSum;
    }

    public static List<List<Double>> array2List(double[][] arr) {
        List<List<Double>> list = new ArrayList<>();
        for (double[] row : arr) {
            List<Double> sublist = new ArrayList<>();
            for (Double num : row) {
                sublist.add(num);
            }
            list.add(sublist);
        }
        return list;
    }

    public static List<Double> array2List(double[] arr) {
        List<Double> list = new ArrayList<>();
        for (double num : arr) {
            list.add(num);
        }
        return list;
    }
}
