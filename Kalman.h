// This file is a part of embed-sensor-fusion project.
// Copyright 2018 Aleksander Gajewski <adiog@brainfuck.pl>.

#pragma once

#include "KalmanImpl.h"
#include <KalmanImpl.h>
#include <SensorFusion.h>
#include <TimeDelta.h>


namespace sensorFusion {
struct Kalman : public SensorFusion
{
    void initialize(const SensorData &sensorData) override
    {
        Float roll = getRoll(sensorData.accelerometer);
        Float pitch = getPitch(sensorData.accelerometer);

        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);

        kalAngleX = roll;
        kalAngleY = pitch;
    }

    FusionData apply(const SensorData &sensorData) override
    {
        Float dt = timeDelta.get();

        Float roll = getRoll(sensorData.accelerometer);
        Float pitch = getPitch(sensorData.accelerometer);

        Float gyroXrate = sensorData.gyroscope[0] / 131.0;  // Convert to deg/s
        Float gyroYrate = sensorData.gyroscope[1] / 131.0;  // Convert to deg/s

        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
        {
            kalmanX.setAngle(roll);
            kalAngleX = roll;
        }
        else
        {
            kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
        }

        if (abs(kalAngleX) > 90)
        {
            gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
        }
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

        return FusionData{{kalAngleX, kalAngleY, 0, 0}};
    }

private:
    KalmanImpl kalmanX;
    KalmanImpl kalmanY;

    Float kalAngleX;
    Float kalAngleY;

    TimeDelta timeDelta;
};
}
