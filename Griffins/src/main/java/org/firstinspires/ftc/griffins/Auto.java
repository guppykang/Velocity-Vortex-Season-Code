package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.FtcChoiceMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous
public class Auto extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {
    HalDashboard halDashboard;
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    public Auto() {
        halDashboard = HalDashboard.getInstance(telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        FtcValueMenu firstDriveDistanceMenu = new FtcValueMenu("First Drive Distance", null, this, 0, 100, 1, 0, "%.0f in");
        FtcValueMenu secondDriveDistanceMenu = new FtcValueMenu("Second Drive Distance", firstDriveDistanceMenu, this,
                0, 100, 1, 48, "%.0f in");
        firstDriveDistanceMenu.setChildMenu(secondDriveDistanceMenu);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance Menu: ", null, this);
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE);

        FtcMenu.walkMenuTree(allianceMenu, this, true);
        halDashboard.resetTelemetryForOpMode();

        Alliance alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        double firstDriveDistance = 48;
        double secondDriveDistance = 48;
        /*telemetry.log().add("First Drive Distance is %.0f in", firstDriveDistance);
        telemetry.log().add("Second Drive Distance is %.0f in", secondDriveDistance);*/
        telemetry.log().add("Alliance is " + alliance);

        waitForStart();
        //autoFunctions.driveStraightSimple((int) (firstDriveDistance * ENCODER_COUNTS_PER_INCH), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        autoFunctions.shoot();
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraightPID(10, AutoFunctions.DriveStraightDirection.FORWARD);
        telemetry.log().add("finished driving straight 1");
        telemetry.update();
        autoFunctions.driveStraightPID(secondDriveDistance - 10, AutoFunctions.DriveStraightDirection.FORWARD);
        telemetry.log().add("Finished driving straight 2");
        telemetry.update();

        if (alliance == Alliance.BLUE_ALLIANCE) {
            autoFunctions.twoWheelTurnPID(90, AutoFunctions.TurnDirection.RIGHT);
            telemetry.log().add("finished turn 1");
            telemetry.update();
            sleep(1000);
            autoFunctions.twoWheelTurnPID(90, AutoFunctions.TurnDirection.LEFT);
            telemetry.log().add("finished turn 2");
            telemetry.update();
        } else {
            autoFunctions.twoWheelTurnPID(90, AutoFunctions.TurnDirection.LEFT);
            telemetry.log().add("finished turn 3");
            telemetry.update();
            sleep(1000);
            autoFunctions.twoWheelTurnPID(90, AutoFunctions.TurnDirection.RIGHT);
            telemetry.log().add("finished turn 4");
            telemetry.update();
        }
        hardware.getIntake().setPower(0.0);
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
