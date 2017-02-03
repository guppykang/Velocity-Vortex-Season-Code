package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.AutoFunctions.DriveStraightDirection;
import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;

/**
 * Created by David on 1/19/2017.
 */
@Autonomous(name = "Beacon Scanning Test", group = "testing")
public class BeaconScan extends LinearOpMode {

    public static double[] scanningSpeeds = {0.05, 0.15};

    public static void wallDrive(double signedPower, RobotHardware hardware) {
        double powerRatio = 0.2 / 0.25;

        hardware.setDrivePower(signedPower, signedPower * powerRatio);
    }

    private static double determineDrivePower(DriveStraightDirection defaultDirection, RobotHardware hardware) {
        BeaconState beaconState = hardware.findBeaconState();

        double drivePower = 0;

        if (beaconState.containsUndefined()) {
            if (beaconState == BeaconState.UNDEFINED_UNDEFINED) {
                drivePower = scanningSpeeds[1] * (defaultDirection == DriveStraightDirection.FORWARD ? 1 : -1);

            } else {
                drivePower = scanningSpeeds[0];
                if ((beaconState.numberState() & 0b11) != BeaconState.UNDEFINED.numberState()) {
                    drivePower *= -1;
                }
            }
        }

        return drivePower;
    }

    public static void scanForBeacon(DriveStraightDirection defaultDirection, RobotHardware hardware, LinearOpMode opMode) {
        double drivePower = determineDrivePower(defaultDirection, hardware);
        double lastDrivePower = drivePower;

        while (opMode.opModeIsActive() && drivePower != 0) {
            lastDrivePower = drivePower;
            wallDrive(drivePower, hardware);
            drivePower = determineDrivePower(defaultDirection, hardware);
        }

        hardware.stopDrive();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        sleep(500);

        scanForBeacon(DriveStraightDirection.BACKWARD, hardware, this);

        sleep(1000);

        hardware.pushButton(hardware.findBeaconState(), BeaconState.RED);

        sleep(3000);

        //autoFunctions.driveStraightPID(36, DriveStraightDirection.FORWARD, 5);

        //scanForBeacon(DriveStraightDirection.FORWARD, hardware);

        //hardware.pushButton(hardware.findBeaconState(), BeaconState.RED);
    }
}
