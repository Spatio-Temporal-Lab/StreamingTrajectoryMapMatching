/* 
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner;


public class HmmProbabilities {

    private final double sigma;

    private final double beta;

    private final double alpha = 5;

    public HmmProbabilities(double sigma, double beta) {
        this.sigma = sigma;
        this.beta = beta;
    }


    public double emissionLogProbability(double distance) {
        return logNormalDistribution(this.sigma, distance);
    }


    public double transitionLogProbability(double routeLength, double linearDistance) {
        double transitionMetric = Math.abs(linearDistance - routeLength);
        if (transitionMetric > 500) {
            return Double.NEGATIVE_INFINITY;
        } else {
            return logExponentialDistribution(this.beta, transitionMetric);
        }

    }


    private static double logNormalDistribution(double sigma, double x) {
        return Math.log(1.0 / (Math.sqrt(2.0 * Math.PI) * sigma)) + (-0.5 * Math.pow(x / sigma, 2));
    }


    private static double logExponentialDistribution(double beta, double x) {
        return Math.log(1.0 / beta) - (x / beta);
    }

    public double directionLogProbability(double diff) {
        return logNormalDistribution(this.alpha, diff);
    }


}
