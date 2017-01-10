package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 1/7/2017.
 */
@Autonomous
public class Shooting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        hardware.getShooter().setPower(.8);
        sleep(1000);
        hardware.setLoaderPower(8.0);
        sleep(5000);
        hardware.getShooter().setPower(0.0);
        hardware.setLoaderPower(0.0);

        sleep(5000);
        autoFunctions.driveStraight((long) (50 * RobotHardware.ENCODER_COUNTS_PER_INCH), AutoFunctions.DriveStraightDirection.FORWARD, 0.7);
    }
}
