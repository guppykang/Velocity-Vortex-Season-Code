package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    private RobotHardware hardware;
    private boolean turretTrackingOn;

    enum DriveStyle {
        TANK_DRIVE,
        ARCADE,
        SPLIT_ARCADE,
        VIDEO_GAME
    }

    private DriveStyle style;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);
        style = DriveStyle.TANK_DRIVE;
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
        double intakeSpeed;
        double loaderPower;
        double targetTurretSpeed;
        RobotHardware.BeaconState beaconPushState;

        if (gamepad1.a) {
            style = DriveStyle.TANK_DRIVE;
        } else if (gamepad1.b) {
            style = DriveStyle.ARCADE;
        } else if (gamepad1.x) {
            style = DriveStyle.SPLIT_ARCADE;
        } else if (gamepad1.y) {
            style = DriveStyle.VIDEO_GAME;
        }

        switch (style) {
            case TANK_DRIVE:
                rightDrivePower = -gamepad1.right_stick_y;
                leftDrivePower = -gamepad1.left_stick_y;
                break;
            case ARCADE:
                rightDrivePower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
                leftDrivePower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
                break;
            case SPLIT_ARCADE:
                rightDrivePower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
                leftDrivePower = -gamepad1.left_stick_y + gamepad1.right_stick_x;
                break;
            case VIDEO_GAME:
                rightDrivePower = gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.left_stick_x;
                leftDrivePower = gamepad1.left_trigger - gamepad1.right_trigger + gamepad1.left_stick_x;
                break;
            default:
                rightDrivePower = 0;
                leftDrivePower = 0;
        }

        if (style != DriveStyle.VIDEO_GAME) {
            intakeSpeed = gamepad1.left_trigger - gamepad1.right_trigger;
        } else {
            intakeSpeed = -gamepad1.right_stick_y;
        }

        if (gamepad1.left_stick_button) {
            rightDrivePower /= 2;
            leftDrivePower /= 2;
        }
        
        if(gamepad2.left_bumper){
            loaderPower = 1.0;
        } else if (gamepad2.right_bumper) {
            loaderPower = -1.0;
        } else {
            loaderPower = 0;
        }

        if (gamepad2.left_trigger != 0) {
            shooterPower = 1;
        } else if (gamepad2.right_bumper) {
            shooterPower = -0.7;
        } else {
            shooterPower = 0;
        }

        double currentTurretSpeed = hardware.getTurretRotation().getPower();
        targetTurretSpeed = -gamepad2.left_stick_x;
        /*if (Math.abs(targetTurretSpeed) < 0.2) {
            currentTurretSpeed = targetTurretSpeed;
        } else {
            if (Math.signum(targetTurretSpeed) != Math.signum(currentTurretSpeed)) {
                currentTurretSpeed = 0;
            }

            if (Math.abs(currentTurretSpeed - targetTurretSpeed) <= 0.05) {
                currentTurretSpeed = targetTurretSpeed;
            } else {
                currentTurretSpeed = currentTurretSpeed + Math.signum(targetTurretSpeed - currentTurretSpeed) * 0.05;
            }
        }*/
        targetTurretSpeed = Math.pow(targetTurretSpeed, 3);

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
        hardware.setTurretRotation(targetTurretSpeed, turretTrackingOn);

        telemetry.addData("Drive Style", style);
        telemetry.addData("Left Drive Speed", leftDrivePower);
        telemetry.addData("Right Drive Speed", rightDrivePower);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Loader Speed", loaderPower);
        telemetry.addData("Shooter Speed", shooterPower);
        telemetry.addData("gamepad 1", gamepad1);
        telemetry.addData("gamepad 2", gamepad2);
    }
}
