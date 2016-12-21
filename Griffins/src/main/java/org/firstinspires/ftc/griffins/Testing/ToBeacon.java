package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 12/20/2016.
 */
@Autonomous
public class ToBeacon extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();
        autoFunctions.shoot();
        sleep(1000);
        autoFunctions.driveStraightPID(14.5, AutoFunctions.DriveStraightDirection.FORWARD);
        sleep(500);
        autoFunctions.twoWheelTurnPID(135, AutoFunctions.TurnDirection.RIGHT);
        sleep(500);
        autoFunctions.driveStraightPID(40.5, AutoFunctions.DriveStraightDirection.BACKWARD);
        sleep(500);
        autoFunctions.twoWheelTurnPID(45, AutoFunctions.TurnDirection.LEFT);
        sleep(1000);
    }
}
