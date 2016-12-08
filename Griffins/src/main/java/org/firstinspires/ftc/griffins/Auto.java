package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous
public class Auto extends LinearOpMode {
    public static final double inchesPerEncoderCount = Math.PI / 140;

    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    public Auto() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);
        waitForStart();
        autoFunctions.driveStraight((int) (40 / inchesPerEncoderCount), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        autoFunctions.shoot();
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraight((int) (32 / inchesPerEncoderCount), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        hardware.getIntake().setPower(0.0);
    }

}
