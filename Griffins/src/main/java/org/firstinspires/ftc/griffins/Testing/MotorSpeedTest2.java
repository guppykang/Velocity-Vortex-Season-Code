package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.griffins.MenuPort.FtcChoiceMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;
import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Iterator;

/**
 * Created by David on 10/22/2016.
 */
@TeleOp
@Disabled
public class MotorSpeedTest2 extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {

    public static final int DEFAULT_RATE_TRACKING_WINDOW = 2000; // In milliseconds

    @Override
    public void runOpMode() throws InterruptedException {
        Iterator<DcMotor> motorIterator = hardwareMap.dcMotor.iterator();
        DcMotor motor = motorIterator.next();
        DcMotor motor2 = null;
        if (motorIterator.hasNext()) {
            motor2 = motorIterator.next();
        }
        ElapsedTime time = new ElapsedTime();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FtcValueMenu rateTrackingWindowMenu = new FtcValueMenu("Rate Tracking Window:", null, this, 500, 10_000, 500, DEFAULT_RATE_TRACKING_WINDOW, "%f ms");
        FtcChoiceMenu motorModeMenu = new FtcChoiceMenu("Motor Mode>", rateTrackingWindowMenu, this);
        FtcValueMenu motorSpeedMenu = new FtcValueMenu("Motor Speed:", motorModeMenu, this, 0, 1, 0.05, .5, "%f");
        motorModeMenu.addChoice("Run Without Encoders", DcMotor.RunMode.RUN_WITHOUT_ENCODER, motorSpeedMenu);
        motorModeMenu.addChoice("Run With Encoders", DcMotor.RunMode.RUN_USING_ENCODER, motorSpeedMenu);
        rateTrackingWindowMenu.setChildMenu(motorModeMenu);


        telemetry.addData("Gamepad", new Func<String>() {
            @Override
            public String value() {
                return gamepad1.toString();
            }
        });

        telemetry.log().add("Procedure: first select motor mode with a and b, run without encoders and run with encoders, respectively.");
        telemetry.log().add("Select power with left joystick, y axis, and press y to select");
        telemetry.log().add("Press x to start rate tracking, motors will run for " + DEFAULT_RATE_TRACKING_WINDOW + " milliseconds.");

        waitForStart();

        while (opModeIsActive()) {
            double speed;
            int encoderPosition1;
            DcMotor.RunMode mode;
            int encoderPosition2 = 0;

            FtcMenu.walkMenuTree(motorModeMenu, this, false);
            mode = (DcMotor.RunMode) motorModeMenu.getCurrentChoiceObject();
            speed = motorSpeedMenu.getCurrentValue();

            motor.setMode(mode);
            if (motor2 != null) {
                motor2.setMode(mode);
            }

            motor.setPower(speed);
            if (motor2 != null) {
                motor2.setPower(-speed);
            }

            while (!gamepad1.a && opModeIsActive()) ;

            encoderPosition1 = motor.getCurrentPosition();
            if (motor2 != null) {
                encoderPosition2 = motor2.getCurrentPosition();
            }
            time.reset();
            sleep(DEFAULT_RATE_TRACKING_WINDOW);

            double rate1 = (motor.getCurrentPosition() - encoderPosition1) / time.seconds();
            double rate2 = 0;
            if (motor2 != null) {
                rate2 = (motor2.getCurrentPosition() - encoderPosition2) / time.seconds();
            }

            telemetry.log().add("motor power, mode: %.2f, %s", motor.getPower(), motor.getMode());
            if (motor2 != null) {
                telemetry.log().add("encoder counts per second: %.2f, %.2f", rate1, rate2);
            } else {
                telemetry.log().add("encoder counts per second: %.2f, %.2f", rate1);
            }
            telemetry.log().add("------------");
            telemetry.update();

            motor.setPower(0);
            motor2.setPower(0);
        }

    }

    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.a;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.b;
    }

    @Override
    public HalDashboard getHalDashboard() {
        return HalDashboard.getInstance(telemetry);
    }
}
