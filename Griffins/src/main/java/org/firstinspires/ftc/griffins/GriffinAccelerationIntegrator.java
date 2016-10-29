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

public class GriffinAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    String log;
    private BNO055IMU.Parameters parameters;
    private Position position;
    private Velocity velocity;
    private Acceleration acceleration;

    public GriffinAccelerationIntegrator() {
        this.parameters = null;
        this.position = null;
        this.velocity = null;
        this.acceleration = null;
    }

    @Override
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.acceleration = null;

        log = "";
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

            //linearAcceleration = round(linearAcceleration);

            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null) {
                Acceleration accelPrev = acceleration;
                Velocity velocityPrev = velocity;

                acceleration = linearAcceleration;

                if (accelPrev.acquisitionTime != 0) {
                    Velocity deltaVelocity = meanIntegrate(acceleration, accelPrev);
                    velocity = plus(velocity, deltaVelocity);
                }

                if (velocityPrev.acquisitionTime != 0) {
                    Position deltaPosition = meanIntegrate(velocity, velocityPrev);
                    position = plus(position, deltaPosition);
                }

                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1e-9, acceleration, velocity, position);
                    log += acceleration.acquisitionTime + ", " + acceleration.xAccel + ", " + acceleration.yAccel +
                            ", " + acceleration.zAccel + "\n";
                }
            } else {
                acceleration = linearAcceleration;
            }
        }
    }

    private Acceleration round(Acceleration acceleration) {
        acceleration.xAccel = ((int) (acceleration.xAccel * 10 + 0.5)) / 10.0;
        acceleration.yAccel = ((int) (acceleration.yAccel * 10 + 0.5)) / 10.0;
        acceleration.zAccel = ((int) (acceleration.zAccel * 10 + 0.5)) / 10.0;


        return acceleration;
    }

    public String getLog() {
        return log;
    }
}
