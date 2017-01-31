package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 1/14/2017.
 */

@Autonomous(name = "Wall Approach Test", group = "testing")
public class WallApproachTest extends LinearOpMode {
    public static void redWallApproach(RobotHardware hardware, AutoFunctions autoFunctions, LinearOpMode opMode) {
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        autoFunctions.twoWheelTurnPID(32, AutoFunctions.TurnDirection.RIGHT, 3);

        autoFunctions.driveStraightPID(11, AutoFunctions.DriveStraightDirection.BACKWARD, 3);

        autoFunctions.twoWheelTurnPID(10, AutoFunctions.TurnDirection.RIGHT, 2);

        autoFunctions.driveStraightPID(15, AutoFunctions.DriveStraightDirection.BACKWARD, 3);

        autoFunctions.twoWheelTurnPID(10, AutoFunctions.TurnDirection.RIGHT, 1);

        autoFunctions.driveStraightPID(15, AutoFunctions.DriveStraightDirection.FORWARD, 4);

        /*hardware.setDrivePower(-.3, -.35);

        LinearOpModeTimeOutFunc timer = new LinearOpModeTimeOutFunc(opMode, 1);
        while (timer.value() && Math.abs(hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 70) > 1) {
            opMode.telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());
            opMode.telemetry.addData("Gyro Target Error", hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 45);
            opMode.telemetry.update();
        }*/

        hardware.stopDrive();
    }

    public static void blueWallApproach(RobotHardware hardware, AutoFunctions autoFunctions, LinearOpMode opMode) {
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        hardware.getIntake().setPower(1); //this really needs to be turned on while driving to the wall, before this method is called.
        autoFunctions.twoWheelTurnPID(30, AutoFunctions.TurnDirection.LEFT, 2);

        autoFunctions.driveStraightPID(11, AutoFunctions.DriveStraightDirection.FORWARD, 1.5);

        autoFunctions.twoWheelTurnPID(10, AutoFunctions.TurnDirection.LEFT, 1);

        autoFunctions.driveStraightPID(49, AutoFunctions.DriveStraightDirection.FORWARD, 3);

        /*hardware.setDrivePower(.3, .35);

        LinearOpModeTimeOutFunc timer = new LinearOpModeTimeOutFunc(opMode, 2);
        while (timer.value() && Math.abs(hardware.getTurretGyro().getIntegratedZValue() - gyroHeading - 45) > 1) {
            opMode.telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());
            opMode.telemetry.addData("Gyro Target Error", hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 45);
            opMode.telemetry.update();
        }*/

        hardware.stopDrive();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        blueWallApproach(hardware, autoFunctions, this);

        sleep(500);

        hardware.setDrivePower(-.35, -.3);

        sleep(1000);

        hardware.stopDrive();
    }


}
