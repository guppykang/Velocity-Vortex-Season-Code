package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.AutoFunctions.DriveStraightDirection;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 12/20/2016.
 */

@Autonomous
public class PIDDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        robot.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);

        waitForStart();

        autoFunctions.driveStraightPID(24, DriveStraightDirection.FORWARD);
        log("finished 24\" drive");
        sleep(2000);
        autoFunctions.driveStraightPID(12, DriveStraightDirection.BACKWARD);
        log("finished -12\" drive");
        sleep(2000);
        autoFunctions.driveStraightPID(12, DriveStraightDirection.FORWARD);
        log("finished 12\" drive");
        sleep(2000);
        autoFunctions.driveStraightPID(48, DriveStraightDirection.FORWARD);
        log("finished 48\" drive");
        sleep(5000);
    }

    public void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }
}
