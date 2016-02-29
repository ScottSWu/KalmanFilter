/**
 * Kalman Filter Implementation
 */

var Sylvester = require('sylvester');
var Matrix = Sylvester.Matrix;
 
/**
 * Create a new Kalman Filter with the given parameters.
 * 
 * @param InitialState      A vector representing the initial state (x0)
 * @param InitialCovariance A matrix representing the initial covariance (P)
 * @param TransitionMatrix  A matrix representing the transition of states (A)
 * @param ObservationMatrix A matrix transforming measurements into states (H)
 * @param ControlMatrix     A matrix representing linear relationships in the
 *                              control vector (B)
 * @param ProcessCovariance
 *                  A matrix representing the process covariance (Q)
 * @param MeasurementCovariance
 *                  A matrix representing the measurement error covariance (R)
 */
var KalmanFilter = function(
    inputInitialState, inputInitialCovariance,
    inputTransitionMatrix, inputObservationMatrix,
    inputControlMatrix, inputProcessCovariance, inputMeasurementCovariance) {
    this.vState = inputInitialState;
    this.mCovariance = inputInitialCovariance;
    this.mTransition = inputTransitionMatrix;
    this.mObservation = inputObservationMatrix;
    this.mControl = inputControlMatrix;
    this.mProcess = inputProcessCovariance;
    this.mMeasurement = inputMeasurementCovariance;
}

/**
 * Update the filter.
 * 
 * @param Measurement       A vector of the current measurement
 * @param Control           A vector of the measurement controls    
 */
KalmanFilter.prototype.update = function(ivMeasurement, ivControl) {
    // Short names
    var xp = this.vState;
    var pp = this.mCovariance;
    
    var A = this.mTransition;
    var AT = A.transpose();
    var B = this.mControl;
    var H = this.mObservation;
    var HT = H.transpose();
    var I = Matrix.I(pp.dimensions().rows);
    var Q = this.mProcess;
    var R = this.mMeasurement;
    
    // Predict the next state
    var vPreState = A.multiply(xp).add(B.multiply(ivControl));
    
    // Predict covariance
    var mPreCovariance = A.multiply(pp).multiply(AT).add(Q);
    
    // Compare prediction with measurement
    var vCmpMeasurement = ivMeasurement.subtract(H.multiply(vPreState));
    
    // Compare prediction covariance with error
    var mCmpCovariance = H.multiply(mPreCovariance).multiply(HT).add(R);
    
    // Compute the Kalman gain
    var mKalman = mPreCovariance.multiply(HT).multiply(mCmpCovariance.inverse());
    
    // Update the state
    this.vState = vPreState.add(mKalman.multiply(vCmpMeasurement));
    
    // Update the covariance
    this.mCovariance = I.subtract(mKalman.multiply(H)).multiply(mPreCovariance);
    
    return this.vState.dup();
}

module.exports = KalmanFilter;
