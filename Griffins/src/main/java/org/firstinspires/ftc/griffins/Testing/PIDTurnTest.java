package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.AutoFunctions.TurnDirection;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 12/20/2016.
 */
@Autonomous
public class PIDTurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        robot.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);

        waitForStart();

        String string = autoFunctions.twoWheelTurnPID(90, TurnDirection.RIGHT);
        log("finished 90 degree turn");
        FileOutput.outputFile("90DegreeTurn.csv", string);
        sleep(2000);
        string = autoFunctions.twoWheelTurnPID(45, TurnDirection.LEFT);
        FileOutput.outputFile("Left45DegreeTurn.csv", string);
        log("finished -45 degree turn");
        sleep(2000);
        string = autoFunctions.twoWheelTurnPID(45, TurnDirection.RIGHT);
        FileOutput.outputFile("Right45DegreeTurn.csv", string);
        log("finished 45 degree turn");
        sleep(2000);
        string = autoFunctions.twoWheelTurnPID(180, TurnDirection.RIGHT);
        FileOutput.outputFile("Left45DegreeTurn.csv", string);
        log("finished 180 degree turn");
        sleep(5000);
    }

    public void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }
}
