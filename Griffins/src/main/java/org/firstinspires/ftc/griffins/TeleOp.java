package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    RobotHardware hardware;
    boolean turretTrackingOn;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready for Teleop", hardware.getTurretGyro().isCalibrating());

    }

    @Override
    public void start() {
        super.start();
        hardware.startTurretTracking();
        turretTrackingOn = false;
    }

    @Override
    public void loop() {
        double leftDrivePower;
        double rightDrivePower;
        double shooterPower;
        double intakeSpeed = 0.0;
        double loaderPower;
        RobotHardware.BeaconState beaconPushState;

        rightDrivePower = Math.pow(-gamepad1.right_stick_y, 3);
        leftDrivePower = Math.pow(-gamepad1.left_stick_y, 3);

        if(gamepad1.left_bumper) {
            intakeSpeed = -1.0;
        }

        if(gamepad1.left_trigger != 0){
            intakeSpeed = gamepad1.left_trigger;
        }

        if(gamepad2.left_bumper){
            loaderPower = -1.0;
        } else if (gamepad2.left_trigger != 0) {
            loaderPower = 1;
        } else {
            loaderPower = 0;
        }

        if (gamepad2.right_bumper) {
            shooterPower = 1;
        } else if (gamepad2.left_bumper) {
            shooterPower = -0.7;
        } else {
            shooterPower = 0;
        }

        hardware.setTurretRotation(Math.pow(-gamepad2.left_stick_x, 3), turretTrackingOn);

        if (gamepad2.right_trigger == 1.0) {
            beaconPushState = RobotHardware.BeaconState.BLUE_RED;
        } else {
            beaconPushState = RobotHardware.BeaconState.UNDEFINED_STATE;
        }

        hardware.getLeftDrive().setPower(leftDrivePower);
        hardware.getRightDrive().setPower(rightDrivePower);
        hardware.getShooter().setPower(shooterPower);
        hardware.getIntake().setPower(intakeSpeed);
        hardware.setLoaderPower(loaderPower);
        hardware.pushButton(beaconPushState);

        telemetry.addData("Left Drive Speed", leftDrivePower);
        telemetry.addData("Right Drive Speed", rightDrivePower);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Loader Speed", loaderPower);
        telemetry.addData("Shooter Speed", shooterPower);
        telemetry.addData("gamepad 1", gamepad1);
        telemetry.addData("gamepad 2", gamepad2);
    }
}
