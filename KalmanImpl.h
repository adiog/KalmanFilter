#include <SensorFusion.h>

class KalmanImpl
{
public:
    using Float = sensorFusion::SensorFusion::Float;
    KalmanImpl();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    Float getAngle(Float newAngle, Float newRate, Float dt);

    void setAngle(Float angle);  // Used to set angle, this should be set as the starting angle
    Float getRate();             // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(Float Q_angle);
    /**
     * setQbias(Float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(Float Q_bias);
    void setRmeasure(Float R_measure);

    Float getQangle();
    Float getQbias();
    Float getRmeasure();

private:
    /* Kalman filter variables */
    Float Q_angle;    // Process noise variance for the accelerometer
    Float Q_bias;     // Process noise variance for the gyro bias
    Float R_measure;  // Measurement noise variance - this is actually the variance of the measurement noise

    Float angle;  // The angle calculated by the Kalman filter - part of the 2x1 state vector
    Float bias;   // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    Float rate;   // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    Float P[2][2];  // Error covariance matrix - This is a 2x2 matrix
};

#endif
