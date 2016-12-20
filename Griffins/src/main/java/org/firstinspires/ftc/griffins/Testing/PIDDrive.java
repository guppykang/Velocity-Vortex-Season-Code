package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by David on 12/19/2016.
 */

public class PIDDrive {

    private RobotHardware hardware;
    private PIDController pid;

    private double difference;

    private PIDController pidRightDrive, pidLeftDrive;

    public void init(){
        hardware = new RobotHardware();

        pidLeftDrive= new  PIDController(1/50.0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double)hardware.getLeftDrive().getCurrentPosition();
            }
        }, hardware.getLeftDrive());

        pidRightDrive = new PIDController(1/50.0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double)hardware.getRightDrive().getCurrentPosition();
            }
        }, hardware.getRightDrive());

        pid = new PIDController(1/50.0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return hardware.getLeftDrive().getPower() - hardware.getRightDrive().getPower();
            }
        }, null);

    }

    public void syncDrives(){

        pidRightDrive.sendPIDOutput();
        pidLeftDrive.sendPIDOutput();

       difference = pid.sendPIDOutput();

        hardware.getLeftDrive().setPower(hardware.getLeftDrive().getPower() - difference);
        hardware.getRightDrive().setPower(hardware.getRightDrive().getPower() + difference);
    }

    public void setTarget(Double target){
        pidRightDrive.setSetPoint(target);
        pidLeftDrive.setSetPoint(target);
        pid.setSetPoint(pid.getSourceVal());
    }

    public void driveToTarget(){
        while(!pidLeftDrive.isOnTarget() && !pidRightDrive.isOnTarget())
            syncDrives();

        while(!pid.isOnTarget())
            syncDrives();
    }
}
