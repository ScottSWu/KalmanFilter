// Kalman Filter noise test (1 independent variable)

// Dependencies
var Sylvester = require('sylvester');
var Matrix = Sylvester.Matrix;
var KalmanFilter = require('./kf.js');

// Parameters
var X = $V([ 0 ]); // Initial state
var P = $M([ 1 ]); // Initial covariance (0, 1?)
var A = $M([ 1 ]); // Constant state transition
var H = $M([ 1 ]); // Input is the same dimension as output
var B = $M([ 0 ]); // No control here
var Q = $M([ 0.0001 ]); // Process covariance, low?
var R = $M([ 0.1 ]); // Measurement covariance

// Initialize the filter
var kf = new KalmanFilter(X, P, A, H, B, Q, R);

// Feed data
var samples = 1000;
for (var i = 0; i < samples; i++) {
    var measurement = Math.random() * 2.0 - 1.0;
    var kalmanV = kf.update($V([ measurement ]), $V([ 0 ]));
    console.log(measurement + "\t" + kalmanV.e(1));
}
