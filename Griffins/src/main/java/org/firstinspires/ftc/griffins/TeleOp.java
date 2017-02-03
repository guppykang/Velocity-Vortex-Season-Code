package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;

/**
 * Created by David on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    private RobotHardware hardware;
    private boolean buttonToggle;
    private boolean turretState;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready for Teleop", !hardware.getTurretGyro().isCalibrating());
    }

    @Override
    public void start() {
        super.start();
        this.resetStartTime();
        hardware.startTurretTracking();
        buttonToggle = gamepad2.x;
        turretState = false;
    }

    @Override
    public void loop() {
        double leftDrivePower;
        double rightDrivePower;
        double shooterPower;
        double intakeSpeed;
        double loaderPower;
        double targetTurretSpeed;
        BeaconState beaconPushState;
        BeaconState alliance = BeaconState.BLUE;
        double beaconPushRatio;
        boolean turretTracking;

        /*if (!buttonToggle && gamepad2.y)
            turretState = !turretState;
        buttonToggle = gamepad2.y;*/

        rightDrivePower = Math.pow(-gamepad1.right_stick_y, 3);
        leftDrivePower = Math.pow(-gamepad1.left_stick_y, 3);

        intakeSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

        if (gamepad1.right_bumper) {
            rightDrivePower = leftDrivePower * .4;
            leftDrivePower *= .5;
        }

        if (gamepad1.left_bumper) {
            rightDrivePower = .20;
            leftDrivePower = .20;
        }

        if (gamepad2.left_bumper) {
            loaderPower = -1.0;
        } else if (gamepad2.left_trigger != 0) {
            loaderPower = 1;
        } else {
            loaderPower = 0;
        }

        if (gamepad2.right_bumper) {
            shooterPower = 1;
        } else if (gamepad2.right_trigger >= 0.5) {
            shooterPower = 0.75;
        } else if (gamepad2.left_bumper) {
            shooterPower = -0.7;
        } else {
            shooterPower = 0;
        }

        double currentTurretSpeed = hardware.getTurretRotation().getPower();
        targetTurretSpeed = gamepad2.left_stick_x;
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
        targetTurretSpeed = Math.pow(targetTurretSpeed, 3) / 3;

        if (gamepad2.x) {
            alliance = BeaconState.BLUE;
            beaconPushState = hardware.findBeaconState();
            beaconPushRatio = 1;
        } else if (gamepad2.b) {
            alliance = BeaconState.RED;
            beaconPushState = hardware.findBeaconState();
            beaconPushRatio = 1;
        } else if (gamepad2.right_stick_x < -0.1) {
            beaconPushState = BeaconState.BLUE_RED;
            beaconPushRatio = -gamepad2.right_stick_x;
        } else if (gamepad2.right_stick_x > 0.1) {
            beaconPushState = BeaconState.RED_BLUE;
            beaconPushRatio = gamepad2.right_stick_x;
        } else {
            beaconPushState = BeaconState.UNDEFINED;
            beaconPushRatio = RobotHardware.BUTTON_PUSHER_RATIO;
        }

        hardware.setDrivePower(leftDrivePower, rightDrivePower);
        hardware.getShooter().setPower(shooterPower);
        hardware.getIntake().setPower(intakeSpeed);
        hardware.setLoaderPower(loaderPower);
        hardware.pushButton(beaconPushState, alliance, beaconPushRatio);
        hardware.setTurretRotation(targetTurretSpeed, turretState);

        int time = (int) getRuntime();

        telemetry.addData("Time(current:remaining)", time + ":" + (120 - time));
        telemetry.addData("Left Drive Speed", leftDrivePower);
        telemetry.addData("Right Drive Speed", rightDrivePower);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Loader Speed", loaderPower);
        telemetry.addData("Shooter Speed", shooterPower);
        telemetry.addData("gamepad 1", gamepad1);
        telemetry.addData("gamepad 2", gamepad2);
        telemetry.addData("left sensor data(a b r g)", hardware.getLeftButtonPusherColorSensor().alpha() + " " +
                hardware.getLeftButtonPusherColorSensor().blue() + " " + hardware.getLeftButtonPusherColorSensor().red() +
                " " + hardware.getLeftButtonPusherColorSensor().green());
        telemetry.addData("Right sensor data(a b r g)", hardware.getRightButtonPusherColorSensor().alpha() + " " +
                hardware.getRightButtonPusherColorSensor().blue() + " " + hardware.getRightButtonPusherColorSensor().red() +
                " " + hardware.getRightButtonPusherColorSensor().green());
    }
}
