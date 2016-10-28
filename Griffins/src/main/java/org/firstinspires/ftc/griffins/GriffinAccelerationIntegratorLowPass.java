package org.firstinspires.ftc.griffins;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

/**
 * Currently a copy of the {@link NaiveAccelerationIntegrator},
 * which provides a very naive implementation of
 * an acceleration integration algorithm. It just does the basic physics.
 * <p/>
 * Todo:
 * One you would actually want to use in a robot would, for example, likely
 * filter noise out the acceleration data or more sophisticated processing.
 * look up what and how to filter noise, -> low pass filter, averages a window to produce value at a point
 * look at better methods for approximating the integral, -> currently uses trapezoid rule, try instead simpson's rule
 */

public class GriffinAccelerationIntegratorLowPass implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    String log;
    private BNO055IMU.Parameters parameters;
    private Position position;
    private Velocity velocity;
    private Acceleration acceleration;

    private LowPassFilter filterX;
    private LowPassFilter filterY;
    private LowPassFilter filterZ;

    public GriffinAccelerationIntegratorLowPass() {
        this.parameters = null;
        this.position = null;
        this.velocity = null;
        this.acceleration = null;
        this.filterX = null;
        this.filterY = null;
        this.filterZ = null;
    }

    @Override
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.acceleration = null;

        this.log = "";
        int windowSize = 10;
        this.filterX = new LowPassFilter(windowSize);
        this.filterY = new LowPassFilter(windowSize);
        this.filterZ = new LowPassFilter(windowSize);
    }

    public Position getPosition() {
        return this.position;
    }

    //------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------

    public Velocity getVelocity() {
        return this.velocity;
    }

    //------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------

    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0) {

            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null) {
                Acceleration previousAcceleration = acceleration;
                Velocity previousVelocity = velocity;

                acceleration = processRawAcceleration(linearAcceleration);

                if (previousAcceleration.acquisitionTime != 0) {
                    Velocity deltaVelocity = meanIntegrate(acceleration, previousAcceleration);
                    velocity = plus(velocity, deltaVelocity);
                }

                if (previousVelocity.acquisitionTime != 0) {
                    Position deltaPosition = meanIntegrate(velocity, previousVelocity);
                    position = plus(position, deltaPosition);
                }

                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - previousAcceleration.acquisitionTime) * 1e-9, acceleration, velocity, position);
                    log += acceleration.acquisitionTime + ", " + acceleration.xAccel + ", " + acceleration.yAccel +
                            ", " + acceleration.zAccel + "\n";
                }
            } else {
                acceleration = linearAcceleration;
            }
        }
    }

    private Acceleration processRawAcceleration(Acceleration linearAcceleration) {
        linearAcceleration.xAccel = filterX.addValue(linearAcceleration.xAccel);
        linearAcceleration.yAccel = filterY.addValue(linearAcceleration.yAccel);
        linearAcceleration.zAccel = filterZ.addValue(linearAcceleration.zAccel);

        return linearAcceleration;
    }
}
