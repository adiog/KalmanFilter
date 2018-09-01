// This file is a part of embed-sensor-fusion project.
// Copyright 2018 Aleksander Gajewski <adiog@brainfuck.pl>.

#include "KalmanImpl.h"
#include <KalmanImpl.h>
#include <SensorFusion.h>
#include <TimeDelta.h>

namespace sensorFusion {
struct Kalman : public SensorFusion
{
    Float getRoll(glm::vec3 accelerometer)
    {
        return glm::degrees(atan2(accelerometer[1], accelerometer[2]));
    }

    Float getPitch(glm::vec3 accelerometer)
    {
        return glm::degrees(atan(-accelerometer[0] / sqrt(accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2])));
    }

    void initialize(const SensorData &sensorData) override
    {
        Float roll = getRoll(sensorData.accelerometer);
        Float pitch = getPitch(sensorData.accelerometer);

        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);

        gyroXangle = roll;
        gyroYangle = pitch;
        compAngleX = roll;
        compAngleY = pitch;
        kalAngleX = roll;
        kalAngleY = pitch;
    }

    Float gyroXangle, gyroYangle;  // Angle calculate using the gyro only
    Float compAngleX, compAngleY;  // Calculated angle using a complementary filter
    Float kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter

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
            compAngleX = roll;
            kalAngleX = roll;
            gyroXangle = roll;
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

        gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
        gyroYangle += gyroYrate * dt;
        //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
        //gyroYangle += kalmanY.getRate() * dt;

        compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
        compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
            gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180)
            gyroYangle = kalAngleY;

        Serial.print(roll);
        Serial.print("\t");
        Serial.print(gyroXangle);
        Serial.print("\t");
        Serial.print(compAngleX);
        Serial.print("\t");
        Serial.print(kalAngleX);
        Serial.print("\t");

        Serial.print("\t");

        Serial.print(pitch);
        Serial.print("\t");
        Serial.print(gyroYangle);
        Serial.print("\t");
        Serial.print(compAngleY);
        Serial.print("\t");
        Serial.print(kalAngleY);
        Serial.print("\t");
        return FusionData{{kalAngleX, kalAngleY, 0}};
    }

private:
    KalmanImpl kalmanX;
    KalmanImpl kalmanY;

    TimeDelta timeDelta;
};
}
