// Kalman Filter noise test (3 dependent variables)

// Dependencies
var Sylvester = require('sylvester');
var Matrix = Sylvester.Matrix;
var KalmanFilter = require('./kf.js');

// Parameters
var dt = 0.01;
var X = $V([ 0, 0, 0, 0, 0, 0 ]); // Initial state (x, y, vx, vy, ax, ay)
var P = Matrix.I(6); // Initial covariance (0, 1?)
var A = $M([
    [ 1, 0, dt, 0, 0.5 * dt * dt, 0 ],
    [ 0, 1, 0, dt, 0, 0.5 * dt * dt ],
    [ 0, 0, 1, 0, dt, 0 ],
    [ 0, 0, 0, 1, 0, dt ],
    [ 0, 0, 0, 0, 1, 0 ],
    [ 0, 0, 0, 0, 0, 1 ],
]); // Constant state transition
var H = $M([
    [ 0, 0, 0, 0, 0, 0 ],
    [ 0, 0, 0, 0, 0, 0 ],
    [ 0, 0, 0, 0, 0, 0 ],
    [ 0, 0, 0, 0, 0, 0 ],
    [ 0, 0, 0, 0, 1, 0 ],
    [ 0, 0, 0, 0, 0, 1 ]
]); // Input is same
var B = Matrix.I(6); // Control?
var Q = Matrix.I(6).multiply(0.001); // Process covariance, low?
var R = Matrix.I(6).multiply(0.2); // Measurement covariance

// Initialize the filter
var kf = new KalmanFilter(X, P, A, H, B, Q, R);

// Feed data
var samples = 3000;
var actual = [ 0, 0, 0, 0, 0, 0 ];
for (var i = 0; i < samples; i++) {
    var ax = 0;
    var ay = 0;
    if (i < 1000) {
        ax = 1;
        ay = -0.5;
    }
    else if (i < 2000) {
        ax = 0;
        ay = 0;
    }
    else if (i < 3000) {
        ax = -1;
        ay = 1;
    }
    
    actual[4] = ax;
    actual[5] = ay;
    actual[2] += ax * dt;
    actual[3] += ay * dt;
    actual[0] += actual[2] * dt + 0.5 * ax * dt * dt;
    actual[1] += actual[3] * dt + 0.5 * ax * dt * dt;
    
    var randx = Math.pow(Math.random(), 2.0) * ((Math.random() < 0.5) ? -1 : 1) * 0.5;
    var randy = Math.pow(Math.random(), 2.0) * ((Math.random() < 0.5) ? -1 : 1) * 0.5;
    ax += randx;
    ay += randy;
    
    var kalmanV = kf.update($V([ 0, 0, 0, 0, ax, ay]), $V([ 0, 0, 0, 0, 0, 0 ]));
    
    console.log(actual.join("\t") + "\t" + kalmanV.e(1) + "\t" + kalmanV.e(2) + "\t" + kalmanV.e(3) + "\t" + kalmanV.e(4) + "\t" + kalmanV.e(5) + "\t" + kalmanV.e(6));
}
