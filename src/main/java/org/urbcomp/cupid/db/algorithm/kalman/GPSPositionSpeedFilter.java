package org.urbcomp.cupid.db.algorithm.kalman;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;


public class GPSPositionSpeedFilter{

    public final KalmanFilter2 filter;


    /*
     * Create a GPS filter that only tracks two dimensions of position and velocity.
     * The inherent assumption is that changes in velocity are randomly distributed around 0.
     * Noise that the higher `observationNoise` is, the more a path will be "smoothed".
     */
    public GPSPositionSpeedFilter(final double processNoise, final double observationNoise){
        //The state model has four dimensions: x, y, dx/dt, dy/dt
        //Each time step we can only observe position, not velocity, so the observation vector has only two dimensions.
        filter = new KalmanFilter2(4, 2);

        //Assuming the axes are rectilinear does not work well at the poles, but it has the bonus that we don't need to convert between
        //lat/long and more rectangular coordinates. The slight inaccuracy of our physics model is not too important.
        final RealMatrix stateTransition = MatrixUtils.createRealIdentityMatrix(filter.getStateDimension());
        filter.setStateTransition(stateTransition);
        setSecondsPerTimeStep(1.);

        //observe (x, y) in each time step
        filter.setObservationModel(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{1., 0., 0., 0.},
                new double[]{0., 1., 0., 0.}
        }));

        //noise in the world
        filter.setProcessNoiseCovariance(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{processNoise, 0., 0., 0.},
                new double[]{0., processNoise, 0., 0.},
                new double[]{0., 0., 1., 0.},
                new double[]{0., 0., 0., 1.}
        }));

        //noise in the observations
        filter.setObservationNoiseCovariance(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{observationNoise, 0.},
                new double[]{0., observationNoise},
        }));

        //initial state
        filter.setInitialStateEstimate(MatrixUtils.createRealMatrix(filter.getStateDimension(), 1));

        //the start position is totally unknown, so give a high variance
        final double trillion = 1_000. * 1_000. * 1_000. * 1_000.;
        final RealMatrix initialEstimateCovariance = MatrixUtils.createRealIdentityMatrix(filter.getStateDimension())
                .scalarMultiply(trillion);
        filter.setInitialEstimateCovariance(initialEstimateCovariance);
    }

    public void setPosition(final double latitude, final double longitude){
        filter.setObservation(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{latitude},
                new double[]{longitude}
        }));
    }

    public void pred(){
        filter.update3();
    }

    public void updatePosition(final double latitude, final double longitude, final long elapsedTime){
        filter.setObservation(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{latitude},
                new double[]{longitude}
        }));
        setSecondsPerTimeStep(elapsedTime);
        filter.update();
    }

    public void updateRealPosition(final double latitude, final double longitude, final long elapsedTime){
        filter.setObservation(MatrixUtils.createRealMatrix(new double[][]{
                new double[]{latitude},
                new double[]{longitude}
        }));
        setSecondsPerTimeStep(elapsedTime);
        filter.update2();
    }


    private void setSecondsPerTimeStep(final double timeLapse){
        filter.setStateTransition(0, 2, timeLapse);
        filter.setStateTransition(1, 3, timeLapse);
    }

    /** Extract filtered position. */
    public double[] getPosition(){
        final double[] latLon = new double[2];
        latLon[0] = filter.getStateEstimate(0, 0);
        latLon[1] = filter.getStateEstimate(1, 0);
        return latLon;
    }

    /** Extract speed with latitude/longitude-per-second units. */
    public double[] getSpeed(){
        final double[] deltaLatLon = new double[2];
        deltaLatLon[0] = filter.getStateEstimate(2, 0);
        deltaLatLon[1] = filter.getStateEstimate(3, 0);
        return deltaLatLon;
    }

    public void setTime(long elapsedTime) {
        setSecondsPerTimeStep(elapsedTime);
    }
}
