package org.firstinspires.ftc.griffins.Testing;

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

    private PIDController pidRightDrive, pidLeftDrive;

    public PIDDrive(RobotHardware hardware) {
        this.hardware = hardware;
        init();
    }

    public void init(){
        pidLeftDrive= new PIDController(1/800.0, 0, 5, new Func<Double>() {
            @Override
            public Double value() {
                return (double)hardware.getLeftDrive().getCurrentPosition();
            }
        }, null);

        pidRightDrive = new PIDController(1/800.0, 0, 5, new Func<Double>() {
            @Override
            public Double value() {
                return (double)hardware.getRightDrive().getCurrentPosition();
            }
        }, null);

        pidDrivingDifference = new PIDController(1/200.0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() - hardware.getRightDrive().getCurrentPosition());
            }
        }, null);

        pidTurningDifference = new PIDController(.1, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition());
            }
        }, null);
    }

    public void syncDrives(){
        double rightPower = pidRightDrive.sendPIDOutput();
        double leftPower = pidLeftDrive.sendPIDOutput();

        if (isTurning) {
            difference = pidTurningDifference.sendPIDOutput();
            leftPower -= difference;
            rightPower += difference;
        } else {
            difference = pidDrivingDifference.sendPIDOutput();
            leftPower += difference;
            rightPower -= difference;
        }


        hardware.getLeftDrive().setPower(leftPower);
        hardware.getRightDrive().setPower(rightPower);
    }

    public void setDriveTarget(double inches){
        pidRightDrive.setSetPoint(hardware.getRightDrive().getCurrentPosition() + inches*RobotHardware.ENCODER_COUNTS_PER_INCH);
        pidLeftDrive.setSetPoint(hardware.getLeftDrive().getCurrentPosition() + inches*RobotHardware.ENCODER_COUNTS_PER_INCH);
        pidDrivingDifference.setSetPoint(pidDrivingDifference.getSourceVal());
        isTurning = false;
    }

    //turns to the right are positive...
    public void setTurnTarget(double degrees) {
        pidRightDrive.setSetPoint(hardware.getRightDrive().getCurrentPosition() - degrees*RobotHardware.ENCODER_COUNTS_PER_ROBOT_DEGREE);
        pidLeftDrive.setSetPoint(hardware.getLeftDrive().getCurrentPosition() + degrees*RobotHardware.ENCODER_COUNTS_PER_ROBOT_DEGREE);
        pidTurningDifference.setSetPoint(pidTurningDifference.getSourceVal());
        isTurning = true;
    }

    public void driveToTarget(Func<Boolean> earlyExitCheck){
        do {
            syncDrives();
        } while(!pidLeftDrive.isOnTarget() && !pidRightDrive.isOnTarget() && earlyExitCheck.value());

        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);
    }
}
