package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.griffins.RobotHardware.ENCODER_COUNTS_PER_ROBOT_DEGREE;

/**
 * Created by David on 12/19/2016.
 */

public class PIDDrive {

    private RobotHardware hardware;
    private PIDController pidDrivingDifference;
    private PIDController pidTurningDifference;

    private double difference;
    private boolean isTurning;

    private PIDController pidTurning, pidDrive;

    public PIDDrive(RobotHardware hardware) {
        this.hardware = hardware;
        init();
    }

    public void init(){
        pidDrive = new PIDController(0.001, 0, 0.003, 22.3, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition()) / 2;
            }
        }, null);

        pidTurning = new PIDController(0.0015 * ENCODER_COUNTS_PER_ROBOT_DEGREE, 0, 0.002 * ENCODER_COUNTS_PER_ROBOT_DEGREE, 1, new Func<Double>() {
            @Override
            public Double value() {
                return (double) hardware.getTurretGyro().getIntegratedZValue();
            }
        }, null);

        pidDrivingDifference = new PIDController(0.001 * ENCODER_COUNTS_PER_ROBOT_DEGREE, 0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double) hardware.getTurretGyro().getIntegratedZValue();
            }
        }, null);

        pidTurningDifference = new PIDController(0, 0, 0, 22.3, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition());
            }
        }, null);
    }

    public void syncDrives(){
        double power;

        if (isTurning) {
            power = pidTurning.sendPIDOutput();
            power = Range.clip(power, -0.5, 0.5);
            difference = pidTurningDifference.sendPIDOutput();

            hardware.setDrivePower(-power - difference, power + difference);
        } else {
            power = pidDrive.sendPIDOutput();
            power = Range.clip(power, -0.5, 0.5);
            difference = pidDrivingDifference.sendPIDOutput();

            hardware.setDrivePower(power + difference, power - difference);
        }


    }

    public void setDriveTarget(double inches){
        pidDrive.setSetPoint(pidDrive.getSourceVal() + inches * RobotHardware.ENCODER_COUNTS_PER_INCH);
        pidDrivingDifference.setSetPoint(pidDrivingDifference.getSourceVal());
        isTurning = false;
    }

    //turns to the left are positive...
    public void setTurnTarget(double degrees) {
        pidTurning.setSetPoint(pidTurning.getSourceVal() + degrees);
        pidTurningDifference.setSetPoint(pidTurningDifference.getSourceVal());
        isTurning = true;
    }

    public void driveToTarget(Func<Boolean> earlyExitCheck, Telemetry telemetry) {
        int exitCounter = 0;
        do {
            syncDrives();
            if (isTurning) {
                if (pidTurning.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }
            } else {
                if (pidDrive.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }
            }

            if (telemetry != null) {
                telemetry.addData("exit counter", exitCounter);
                telemetry.addData("exit condition value", exitCounter <= 10 && earlyExitCheck.value());
                telemetry.addData("source value", isTurning ? pidTurning.getSourceVal() : pidDrive.getSourceVal());
                telemetry.addData("target", isTurning ? pidTurning.getSetPoint() : pidDrive.getSetPoint());
                telemetry.addData("error", isTurning ? pidTurning.getError() : pidDrive.getError());
                telemetry.update();
            }
        } while (exitCounter <= 100 && earlyExitCheck.value());

        hardware.stopDrive();
    }

    public void driveToTarget(Func<Boolean> booleanFunc) {
        driveToTarget(booleanFunc, null);
    }
}