package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;

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
        pidDrive = new PIDController(0.001, 0, 0.003, new Func<Double>() {
            @Override
            public Double value() {
                return (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition()) / 2.0;
            }
        }, null);

        pidTurning = new PIDController(0.001, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (0 - hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition()) / 2.0;
            }
        }, null);

        pidDrivingDifference = new PIDController(.001, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() - hardware.getRightDrive().getCurrentPosition());
            }
        }, null);

        pidTurningDifference = new PIDController(0, 0, 0, new Func<Double>() {
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

            hardware.getLeftDrive().setPower(-power - difference);
            hardware.getRightDrive().setPower(power + difference);
        } else {
            power = pidDrive.sendPIDOutput();
            power = Range.clip(power, -0.5, 0.5);
            difference = pidDrivingDifference.sendPIDOutput();

            hardware.getLeftDrive().setPower(power + difference);
            hardware.getRightDrive().setPower(power - difference);
        }


    }

    public void setDriveTarget(double inches){
        pidDrive.setSetPoint(pidDrive.getSourceVal() + inches * RobotHardware.ENCODER_COUNTS_PER_INCH);
        pidDrivingDifference.setSetPoint(pidDrivingDifference.getSourceVal());
        isTurning = false;
    }

    //turns to the left are positive...
    public void setTurnTarget(double degrees) {
        pidTurning.setSetPoint(pidTurning.getSourceVal() + degrees * RobotHardware.ENCODER_COUNTS_PER_ROBOT_DEGREE);
        pidTurningDifference.setSetPoint(pidTurningDifference.getSourceVal());
        isTurning = true;
    }

    public void driveToTarget(Func<Boolean> earlyExitCheck){
        do {
            syncDrives();
        } while ((!pidDrive.isOnTarget() || !pidTurning.isOnTarget()) && earlyExitCheck.value());

        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);
    }
}