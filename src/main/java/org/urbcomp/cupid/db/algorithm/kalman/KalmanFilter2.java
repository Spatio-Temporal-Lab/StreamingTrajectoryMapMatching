package org.urbcomp.cupid.db.algorithm.kalman;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;


public class KalmanFilter2 {

    private final int stateDimension;
    private final int observationDimension;

    //state-transition model (the kinematic equations, predicts how the state would evolve naturally without input), F_k
    private RealMatrix stateTransition;
    //observation mode (which things in the state matrix are actually measured), H_k
    private RealMatrix observationModel;
    //covariance of the process noise (how bad the prediction model is at telling us what is actually going on), Q_k
    private RealMatrix processNoiseCovariance;
    //covariance of the observation noise (how much noise is inherent to the measurement -- not what is actually happening --), R_k
    private RealMatrix observationNoiseCovariance;

    //the observation is modified by the user before every time step
    //observation of the true state x_k, z_k
    private RealMatrix observation;

    //x-hat_k|k-1
    private RealMatrix predictedState;
    //P-hat_k|k-1
    private RealMatrix predictedEstimateCovariance;
    //x-hat_k|k
    private RealMatrix stateEstimate;
    //a posteriori estimate covariance matrix (a measure of the estimated accuracy of the state estimate), P_k|k
    private RealMatrix estimateCovariance;


    public KalmanFilter2(final int stateDimension, final int observationDimension){
        this.stateDimension = stateDimension;
        this.observationDimension = observationDimension;
    }

    public int getStateDimension(){
        return stateDimension;
    }

    public void setStateTransition(final RealMatrix stateTransition){
        if(stateTransition.getRowDimension() != stateDimension || stateTransition.getColumnDimension() != stateDimension)
            throw new IllegalArgumentException("State transition matrix must be of dimension "
                    + stateDimension + "×" + stateDimension);

        this.stateTransition = stateTransition;
    }

    public void setStateTransition(final int row, final int column, final double value){
        stateTransition.setEntry(row, column, value);
    }

    public void setObservationModel(final RealMatrix observationModel){
        if(observationModel.getRowDimension() != observationDimension || observationModel.getColumnDimension() != stateDimension)
            throw new IllegalArgumentException("Observation model matrix must be of dimension "
                    + observationDimension + "×" + stateDimension);

        this.observationModel = observationModel;
    }

    public void setProcessNoiseCovariance(final RealMatrix processNoiseCovariance){
        if(processNoiseCovariance.getRowDimension() != stateDimension || processNoiseCovariance.getColumnDimension() != stateDimension)
            throw new IllegalArgumentException("Process noise covariance matrix must be of dimension "
                    + stateDimension + "×" + stateDimension);

        this.processNoiseCovariance = processNoiseCovariance;
    }

    public void setObservationNoiseCovariance(final RealMatrix observationNoiseCovariance){
        if(observationNoiseCovariance.getRowDimension() != observationDimension
                || observationNoiseCovariance.getColumnDimension() != observationDimension)
            throw new IllegalArgumentException("Observation noise covariance matrix must be of dimension "
                    + observationDimension + "×" + observationDimension);

        this.observationNoiseCovariance = observationNoiseCovariance;
    }

    public void setInitialStateEstimate(final RealMatrix stateEstimate){
        if(stateEstimate.getRowDimension() != stateDimension
                || stateEstimate.getColumnDimension() != 1)
            throw new IllegalArgumentException("Initial state estimate matrix must be of dimension " + stateDimension + "×" + 1);

        this.stateEstimate = stateEstimate;
    }

    public void setInitialEstimateCovariance(final RealMatrix estimateCovariance){
        if(estimateCovariance.getRowDimension() != stateDimension
                || estimateCovariance.getColumnDimension() != stateDimension)
            throw new IllegalArgumentException("Initial estimate covariance matrix must be of dimension "
                    + stateDimension + "×" + stateDimension);

        this.estimateCovariance = estimateCovariance;
    }

    public void setObservation(final RealMatrix observation){
        if(observation.getRowDimension() != observationDimension
                || observation.getColumnDimension() != 1)
            throw new IllegalArgumentException("Observation matrix must be of dimension " + observationDimension + "×" + 1);

        this.observation = observation;
    }

    public double getStateEstimate(final int row, final int column){
        return stateEstimate.getEntry(row, column);
    }

    public void setStateEstimate(final int row, final int col, final double var){
        stateEstimate.setEntry(row, col, var);
    }

    /**
     * Runs one time step of prediction and estimation.
     * <p>
     * Before each time step of running this, set <code>f.observation</code> to be the next time step's observation.
     * </p>
     * <p>
     * Before the first step, define the model by setting: <code>f.state_transition</code>, <code>f.observation_model</code>,
     * <code>f.process_noise_covariance</code>, and <code>f.observation_noise_covariance</code>.
     * </p>
     * <p>
     * It is also advisable to initialize with reasonable guesses for <code>f.state_estimate</code> and <code>f.estimate_covariance</code>.
     * </p>
     */
    public void update(){
        predict();
        estimate();
    }

    public void update2(){
        estimate();
    }

    public void update3(){ predict();}


    private void predict(){
        //predicted (a priori) state estimate, x-hat_k|j-1 = F_k * x_k-1|k-1 + B_k * u_k, where B_k is the control matrix, and u_k is the
        //	control variable
        predictedState = stateTransition.multiply(stateEstimate);

        //predicted (a priori) state estimate covariance, P-hat_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
        predictedEstimateCovariance = stateTransition.multiply(estimateCovariance)
                .multiply(stateTransition.transpose())
                .add(processNoiseCovariance);
    }

    private void estimate(){
        //innovation, or measurement pre-fit residual, y-tilde_k = z_k - H_k * x-hat_k|k-1
        final RealMatrix innovation = observation.subtract(observationModel.multiply(predictedState));

        final RealMatrix verticalScratch = predictedEstimateCovariance.multiply(observationModel.transpose());
        //innovation, or pre-fit residual, covariance, S_k = H_k * P-hat_k|k-1 * H_k^T + R_k
        final RealMatrix innovationCovariance = observationModel.multiply(verticalScratch)
                .add(observationNoiseCovariance);

        //calculate the optimal Kalman gain, K_k = P-hat_k|k-1 * H_k^T * S_k^-1
        final RealMatrix optimalGain = verticalScratch.multiply(MatrixUtils.inverse(innovationCovariance));

        //updated (a posteriori) state estimate, x_k|k = x-hat_k|k-1 + K_k * y-tilde_k
        stateEstimate = predictedState.add(optimalGain.multiply(innovation));

        //updated (a posteriori) estimate covariance, P_k|k = (I - K_k * H_k) * P-hat_k|k-1
        estimateCovariance = MatrixUtils.createRealIdentityMatrix(stateDimension)
                .subtract(optimalGain.multiply(observationModel))
                .multiply(predictedEstimateCovariance);
    }

}
