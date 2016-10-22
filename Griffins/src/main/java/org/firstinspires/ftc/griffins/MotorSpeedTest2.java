package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by David on 10/22/2016.
 */
@TeleOp

public class MotorSpeedTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.iterator().next();
        ElapsedTime time = new ElapsedTime();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            double speed;
            int encoderPosition = motor.getCurrentPosition();

            while (!gamepad1.a && !gamepad1.b && opModeIsActive()) {
                telemetry.addData("press a to select", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("press b to select", DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.a) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.b) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            while (!gamepad1.y && opModeIsActive()) {
                telemetry.addData("motor mode", motor.getMode());
                telemetry.addData("speed to selected (y to select)", -gamepad1.left_stick_y);
                telemetry.update();
            }

            speed = -gamepad1.left_stick_y;

            motor.setPower(speed);

            while (!gamepad1.x && opModeIsActive()) {
                telemetry.addData("motor mode", motor.getMode());
                telemetry.addData("speed selected", -gamepad1.left_stick_y);
                telemetry.addData("press x to start test", true);
                telemetry.update();
            }

            if (!opModeIsActive()) {
                encoderPosition = motor.getCurrentPosition();
                time.reset();
                sleep(1000);
            }

            double rate = (motor.getCurrentPosition() - encoderPosition) / time.seconds();

            telemetry.log().add("motor power: " + motor.getPower());
            telemetry.log().add("motor mode: " + motor.getMode());
            telemetry.log().add("encoder counts per second: %.2f", rate);
            telemetry.log().add("------------");
            telemetry.update();

            motor.setPower(0);
        }

    }
}
