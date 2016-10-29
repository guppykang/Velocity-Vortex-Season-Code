package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Iterator;

/**
 * Created by David on 10/22/2016.
 */
@TeleOp

public class MotorSpeedTest2 extends LinearOpMode {

    public static final int RATE_TRACKING_WINDOW = 2000; // In milliseconds

    @Override
    public void runOpMode() throws InterruptedException {
        Iterator<DcMotor> motorIterator = hardwareMap.dcMotor.iterator();
        DcMotor motor = motorIterator.next();
        DcMotor motor2 = motorIterator.next();
        ElapsedTime time = new ElapsedTime();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Gamepad", new Func<String>() {
            @Override
            public String value() {
                return gamepad1.toString();
            }
        });

        telemetry.log().add("Procedure: first select motor mode with a and b, run without encoders and run with encoders, respectively.");
        telemetry.log().add("Select power with left joystick, y axis, and press y to select");
        telemetry.log().add("Press x to start rate tracking, motors will run for " + RATE_TRACKING_WINDOW + " milliseconds.");

        waitForStart();

        while (opModeIsActive()) {
            double speed;
            int encoderPosition1;
            int encoderPosition2;

            while (!gamepad1.a && !gamepad1.b && opModeIsActive()) ;
            if (gamepad1.a) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.b) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            while (!gamepad1.y && opModeIsActive()) ;

            speed = -gamepad1.left_stick_y;

            motor.setPower(speed);
            motor2.setPower(-speed);

            while (!gamepad1.x && opModeIsActive()) ;

            encoderPosition1 = motor.getCurrentPosition();
            encoderPosition2 = motor2.getCurrentPosition();
            time.reset();
            sleep(RATE_TRACKING_WINDOW);

            double rate1 = (motor.getCurrentPosition() - encoderPosition1) / time.seconds();
            double rate2 = (motor2.getCurrentPosition() - encoderPosition2) / time.seconds();


            telemetry.log().add("motor power, mode: %.2f, %s", motor.getPower(), motor.getMode());
            telemetry.log().add("encoder counts per second: %.2f, %.2f", rate1, rate2);
            telemetry.log().add("------------");
            telemetry.update();

            motor.setPower(0);
            motor2.setPower(0);
        }

    }
}
