package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 11/12/2016 for Josh.
 * Go ahead and start implementing a basic drive base.
 */

public class Teleop extends OpMode {
    RobotHardware roboto;

    @Override
    public void init() {
        roboto = new RobotHardware();
        roboto.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        double left;
        double right;
        double shooterspeed;
        double intake;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        shooterspeed = gamepad2.right_trigger;
        intake = gamepad2.left_trigger;
        roboto.getLeftDrive().setPower(left);
        roboto.getRightDrive().setPower(right);
        roboto.getShooter().setPower(shooterspeed);
        roboto.getIntake().setPower(intake);

        telemetry.addData("left stick speed", left);
        telemetry.addData("right stick speed", right);
        telemetry.addData("right trigger speed", shooterspeed);
        telemetry.addData("intake speed", intake);
    }
}


