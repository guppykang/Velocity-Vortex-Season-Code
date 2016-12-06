package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    RobotHardware hardware;
    int turretHeadingTarget;
    boolean turretTrackingOn;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        turretHeadingTarget = hardware.getTurretGyro().getIntegratedZValue();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready for Teleop", hardware.getTurretGyro().isCalibrating());

    }

    @Override
    public void start() {
        super.start();
        hardware.startTurretTracking();
        turretTrackingOn = true;
    }

    @Override
    public void loop() {
        double leftVal = -gamepad1.left_stick_y;
        double rightVal = -gamepad1.right_stick_y;
        RobotHardware.BeaconState beaconPushState;

        if (turretTrackingOn) {
            hardware.setTurretRotation((int) (-gamepad2.left_stick_x * 2));
        }
        boolean intakeOn = false;
        double intakeSpeed = 0.0;

        if (gamepad2.a) {
            hardware.getTurretRotation().setPower(gamepad2.left_stick_x);
            hardware.setTurretHeadingTarget(hardware.getTurretGyro().getIntegratedZValue());
        }

        if(gamepad1.left_bumper) {
            intakeSpeed = -1.0;
            intakeOn = true;
        }

        if(gamepad1.left_trigger != 0){
            intakeSpeed = gamepad1.left_trigger;
            intakeOn = true;
        }

        if(gamepad2.left_bumper){
            hardware.setLoaderPower(-1.0);
            hardware.getShooter().setPower(-0.2);
        } else if (gamepad2.left_trigger != 0) {
            hardware.setLoaderPower(1);
        } else {
            hardware.setLoaderPower(0);
        }

        if (gamepad2.right_bumper) {
            hardware.getShooter().setPower(1);
        } else {
            hardware.getShooter().setPower(0);
        }

        hardware.getIntake().setPower(intakeSpeed);

        hardware.setTurretRotation((int) (gamepad2.right_stick_x * 5));

        if(gamepad2.right_trigger == 1.0){
            beaconPushState = RobotHardware.BeaconState.BLUE_RED;
        }else
            beaconPushState = RobotHardware.BeaconState.UNDEFINED_STATE;


        hardware.getLeftDrive().setPower(leftVal);
        hardware.getRightDrive().setPower(rightVal);
        hardware.pushButton(beaconPushState);

        telemetry.addData("Left Drive Speed", leftVal);
        telemetry.addData("Right Drive Speed", rightVal);
        telemetry.addData("Intake On?", intakeOn);
        telemetry.addData("Intake Speed", intakeSpeed);
    }
}
