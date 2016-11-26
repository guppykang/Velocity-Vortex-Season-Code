package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    RobotHardware hardware;
    int turretHeadingTarget;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        turretHeadingTarget = hardware.getTurretGyro().getIntegratedZValue();
    }

    @Override
    public void loop() {
        double leftVal = -gamepad1.left_stick_y;
        double rightVal = -gamepad1.right_stick_y;

        double angle = -gamepad2.left_stick_y;

        hardware.getLeftDrive().setPower(leftVal);
        hardware.getRightDrive().setPower(rightVal);

        hardware.setTurretGuidePosition(angle);

        boolean intakeOn = false;
        double intakeSpeed = 0.0;

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
            intakeSpeed = -1.0;
            intakeOn = true;
        }

        if(gamepad2.left_trigger != 0){
            hardware.setLoaderPower(gamepad2.left_trigger);
            hardware.getShooter().setPower(1.0);
        }else
            hardware.getShooter().setPower(0.0);

        hardware.getIntake().setPower(intakeSpeed);

        turretHeadingTarget += gamepad1.right_stick_x * 5;
        double turretError = -(turretHeadingTarget - hardware.getTurretGyro().getIntegratedZValue());
        double turretSpeed = turretError / 100;
        turretSpeed = Range.clip(turretSpeed, -.5, .5);

        if (hardware.getTurretRotation().getCurrentPosition() > RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, -1, 0);
            if (turretError > 20) {
                turretHeadingTarget += 360;
                //turretSpeed = -1;
            }

        } else if (hardware.getTurretRotation().getCurrentPosition() < -RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, 0, 1);
            if (turretError < -20) {
                turretHeadingTarget -= 360;
                // turretSpeed = 1;
            }

        }

        hardware.getTurretRotation().setPower(turretSpeed);

        if(gamepad2.right_trigger == 1.0){
            hardware.pushButton();
        }else
            hardware.pushButton(); //make so that there are different parameters and retracts

        telemetry.addData("Left Drive Speed", leftVal);
        telemetry.addData("Right Drive Speed", rightVal);
        telemetry.addData("Turret Angle", angle);
        telemetry.addData("Intake On?", intakeOn);
        telemetry.addData("Intake Speed", intakeSpeed);
    }
}
