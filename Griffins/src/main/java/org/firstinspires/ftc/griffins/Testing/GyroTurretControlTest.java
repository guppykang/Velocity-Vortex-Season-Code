package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;

import static org.firstinspires.ftc.griffins.RobotHardware.TURRET_GYRO;
import static org.firstinspires.ftc.griffins.RobotHardware.TURRET_ROTATION_MOTOR;

/**
 * Created by David on 11/22/2016.
 */

@TeleOp(name = "gyro turret control", group = "test")
@Disabled
public class GyroTurretControlTest extends OpMode {

    int turretHeadingTarget;

    DcMotor turretRotation;
    ModernRoboticsI2cGyro turretGyro;

    @Override
    public void init() {

        turretRotation = hardwareMap.get(DcMotor.class, TURRET_ROTATION_MOTOR);
        turretRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretGyro = (ModernRoboticsI2cGyro) hardwareMap.get(GyroSensor.class, TURRET_GYRO);
        turretGyro.calibrate();  //look at z axis scaling coefficient when available
        turretGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        turretRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gamepad1.setJoystickDeadzone(0.1f);

        turretHeadingTarget = turretGyro.getIntegratedZValue();

        telemetry.log().add("Hardware Initialized, ready to start");

        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        //turret on gamepad 1 right x axis, attempts to maintain heading relative to field
        turretHeadingTarget += gamepad1.right_stick_x * 5;
        double turretError = -(turretHeadingTarget - turretGyro.getIntegratedZValue());
        double turretSpeed = turretError / 100;
        turretSpeed = Range.clip(turretSpeed, -.5, .5);

        if (turretRotation.getCurrentPosition() > RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, -1, 0);
            if (turretError > 20) {
                turretHeadingTarget += 360;
                //turretSpeed = -1;
            }

        } else if (turretRotation.getCurrentPosition() < -RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, 0, 1);
            if (turretError < -20) {
                turretHeadingTarget -= 360;
                // turretSpeed = 1;
            }

        }

        turretRotation.setPower(turretSpeed);

        telemetry.addData("Turret Heading Current|Target", turretGyro.getIntegratedZValue() + "|" + turretHeadingTarget);
        telemetry.addData("Turret Error", turretError);
        telemetry.addData("Turret Speed", turretSpeed);
        telemetry.addData("Turret Counts", turretRotation.getCurrentPosition());
    }
}
