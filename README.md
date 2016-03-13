# KalmanFilter

Simple straightforward implementation of the Kalman Filter for nodejs. The `KalmanFilter` class is within `kf.js`, and two examples are `test_kf1.js` and `test_kf2.js`.

`test_kf1.js` uses a constant state model, and feeds uniform random data to the filter.

`test_kf2.js` imitates an accelerometer and uses the filter to smooth acceleration while computing velocity and position.
