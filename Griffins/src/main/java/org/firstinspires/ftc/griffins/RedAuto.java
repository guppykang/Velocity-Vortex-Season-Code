package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;
import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;
import org.firstinspires.ftc.griffins.Testing.WallApproachTest;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Red Beacon Auto", group = "Auto")
public class RedAuto extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {
    HalDashboard halDashboard;
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    public RedAuto() {
        halDashboard = HalDashboard.getInstance(telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        Alliance alliance = Alliance.RED_ALLIANCE;
        telemetry.log().add("Alliance is " + alliance);

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        //shoot two particles
        //autoFunctions.shoot();
        telemetry.log().add("Finished Shooting");
        telemetry.update();

        //drive straight a little to get off wall in order to turn
        autoFunctions.driveStraightPID(10, AutoFunctions.DriveStraightDirection.FORWARD, 5);
        telemetry.log().add("Off the Wall");
        telemetry.update();

        hardware.getIntake().setPower(1);
        //turn so facing toward beacon
        autoFunctions.twoWheelTurnPID(119, AutoFunctions.TurnDirection.RIGHT, 8);
        telemetry.log().add("Turned towards beacon");
        telemetry.update();
        hardware.getIntake().setPower(0);

        //drive toward beacon wall
        autoFunctions.driveStraightPID(59, AutoFunctions.DriveStraightDirection.BACKWARD, 3);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        autoFunctions.driveStraightPID(2, AutoFunctions.DriveStraightDirection.FORWARD, 1);

        WallApproachTest.redWallApproach(hardware, autoFunctions, this);

        hardware.pushButton(hardware.findBeaconState(), BeaconState.RED);

        sleep(2000);

        hardware.pushButtonFullExtension(BeaconState.UNDEFINED, BeaconState.RED);

        BeaconState state = hardware.findBeaconState();
        if (state != BeaconState.RED_RED) {
            if (getRuntime() >= 28) {
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else if (getRuntime() >= 20) {
                sleep((long) ((28 - getRuntime()) * 1000));
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else {
                sleep(6000);
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            }

            sleep(2000);
            hardware.pushButton(BeaconState.UNDEFINED, BeaconState.RED);
        }

        autoFunctions.twoWheelTurnPID(3, AutoFunctions.TurnDirection.LEFT, 1.5);
        autoFunctions.driveStraightPID(48, AutoFunctions.DriveStraightDirection.BACKWARD, 3);

        hardware.pushButtonFullExtension(hardware.findBeaconState(), BeaconState.RED);

        sleep(2000);

        hardware.pushButton(BeaconState.UNDEFINED, BeaconState.RED);

        hardware.setDrivePower(.2, .4);
        hardware.getIntake().setPower(1);

        sleep(2000);

        hardware.stopDrive();
        hardware.getIntake().setPower(0);

        /*//"parallel parking"
        autoFunctions.curveDriveShort(-(long) (3 / hardware.INCHES_PER_ENCODER_COUNT), -(long) (12.5 / hardware.INCHES_PER_ENCODER_COUNT), .1, .9);
        telemetry.log().add("Straightened out against wall");
        telemetry.update();

        //drive back to first beacon
        autoFunctions.driveStraight((long)(34/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.BACKWARD, .5);
        telemetry.log().add("Arrived at first beacon");
        telemetry.update();

        //button pusher works its magic
       if(opModeIsActive()){
         hardware.pushButton(hardware.findBeaconState(), RobotHardware.BeaconState.RED_RED);
       }
        telemetry.log().add("Pushed first button");
        telemetry.update();

        //drive straight until in front of second beacon
        autoFunctions.driveStraight((long)(50/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        telemetry.log().add("Arrived at second beacon");
        telemetry.update();

        //button pusher works its magic pt 2
        if(opModeIsActive()){
            hardware.pushButton(hardware.findBeaconState(), RobotHardware.BeaconState.RED_RED);
        }
        telemetry.log().add("Pushed second button");
        telemetry.update();*/

    }

    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.a;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.b;
    }

    @Override
    public HalDashboard getHalDashboard() {
        halDashboard.resetTelemetryForHalDashboard();
        return halDashboard;
    }

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}