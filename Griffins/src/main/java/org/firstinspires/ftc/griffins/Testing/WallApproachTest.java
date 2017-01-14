package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.Navigation.LinearOpModeTimeOutFunc;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 1/14/2017.
 */

@Autonomous
public class WallApproachTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        hardware.getTurretGyro().resetZAxisIntegrator();

        autoFunctions.twoWheelTurn(-30);

        hardware.setDrivePower(-.3, -.4);

        LinearOpModeTimeOutFunc timer = new LinearOpModeTimeOutFunc(this, 2);
        while (timer.value() && Math.abs(hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 45) > 1) {
            telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());
        }

        hardware.stopDrive();

        sleep(500);

        hardware.setDrivePower(-.35, -.3);

        sleep(1000);

        hardware.stopDrive();
    }
}
