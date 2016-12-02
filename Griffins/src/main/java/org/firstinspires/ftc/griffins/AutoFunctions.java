package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 11/28/2016.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AutoFunctions {
    private LinearOpMode linearOpMode;
    private RobotHardware hardware;

    public AutoFunctions(RobotHardware hardware, LinearOpMode linearOpMode) {
        this.hardware = hardware;
        this.linearOpMode = linearOpMode;
    }

    public void oneWheelTurn(DcMotor turningMotor, double angle) throws InterruptedException {
        double minimumPower = .2;
        int gyroTarget = (int)getZAngle();

        ElapsedTime timeout = new ElapsedTime();
        double drivePower = 1.0;
        do {
            linearOpMode.idle(); //replace with idle, check that while loops call opmode is active
            int headingError = (gyroTarget - (int)getZAngle());

            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            RobotLog.i("-----------------------");
            turningMotor.setPower(drivePower);
            drivePower = headingError / (2 * angle);
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(getZAngle()) > 0 && timeout.time() < 5); //DAVID: can't find new getRobotRotationGyro method

        //stop driving
        linearOpMode.idle();
        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);

        //send any late signals
        linearOpMode.idle();
    }

    public void twoWheelTurn(double angle) throws InterruptedException {

        //curve around to face the ramp
        int gyroTarget;
        gyroTarget = (int)getZAngle() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double minimumPower = .10;
        double drivePower = 1.0;
        do {
            linearOpMode.idle();
            int headingError = (gyroTarget - (int)getZAngle());
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            hardware.getLeftDrive().setPower(drivePower);
            hardware.getRightDrive().setPower(-drivePower);
            drivePower = headingError / (2 * Math.abs(angle));
            drivePower = Range.clip(drivePower, -1, 1);
            RobotLog.i("2w -----------------------");
            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("target, current", gyroTarget + ", " + getZAngle());
            linearOpMode.telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(getZAngle() - gyroTarget) > 0 && timeout.time() < 5);

        //stop motors
        linearOpMode.idle();
        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);

        //send any late signals
        linearOpMode.idle();
    }

    public void driveStraight(long encoderCount, DriveStraightDirection direction, double power) throws InterruptedException {
        double minimumPower = .1;
        if (power < 0) {
            throw new IllegalArgumentException("Power must be greater than 0");
        } else if (power > 1) {
            throw new IllegalArgumentException("Power must be less than 1");
        }
        if (encoderCount < 0) {
            throw new IllegalArgumentException(" Encoder count must be greater than 0");
        }
        ElapsedTime timeout = new ElapsedTime();
        //encoder target
        long encoderTarget;
        RobotLog.i("Drive Straight --------------");
        RobotLog.i("Direction: " + direction);
        RobotLog.i("Power: " + power);
        RobotLog.i("Encoder Counts: " + encoderCount);

        long leftEncoderOffset = hardware.getLeftDrive().getCurrentPosition();
        long rightEncoderOffset = hardware.getRightDrive().getCurrentPosition();

        if (direction == DriveStraightDirection.BACKWARD) {
            power = -power;
            encoderTarget = hardware.getLeftDrive().getCurrentPosition() - encoderCount;
        } else {
            encoderTarget = hardware.getLeftDrive().getCurrentPosition() + encoderCount;
        }
        //drive forward, clearing front of ramp
        timeout.reset();
        boolean stopCondition;
        do {

            //how much the left motor is ahead of the right motor.
            long encoderDifference = (hardware.getLeftDrive().getCurrentPosition() - leftEncoderOffset) - (hardware.getRightDrive().getCurrentPosition() - rightEncoderOffset);
            long error = hardware.getLeftDrive().getCurrentPosition() - encoderDifference / 2;

            //calculate using encoder difference
            double powerOffset = 0;

            power = encoderCount / (2 * error);

            if (Math.abs(power) < minimumPower) {
                power = minimumPower * Math.signum(power);
            }

            RobotLog.i("DriveStraight loop------");
            RobotLog.i("Encoder Counts to go: " + error);
            RobotLog.i("Power: " + power);
            RobotLog.i("Encoder difference: " + encoderDifference);
            RobotLog.i("Power offset: " + powerOffset);
            linearOpMode.telemetry.addData("Encoder Counts to go", Math.abs(hardware.getLeftDrive().getCurrentPosition() - encoderTarget));

            linearOpMode.idle();
            hardware.getLeftDrive().setPower(power - powerOffset);
            hardware.getRightDrive().setPower(power + powerOffset);

            stopCondition = Math.abs(hardware.getLeftDrive().getCurrentPosition() - encoderTarget) < 3;
        } while (stopCondition && timeout.time() < 10);

        //stop motors
        linearOpMode.idle();
        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);

        //send any late signals
        linearOpMode.idle();
    }

    public enum DriveStraightDirection {FORWARD, BACKWARD}

    public float getZAngle(){
        return hardware.getRobotTracker().getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

    }
}