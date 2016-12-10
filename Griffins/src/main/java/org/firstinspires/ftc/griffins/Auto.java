package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;

import static org.firstinspires.ftc.griffins.RobotHardware.ENCODER_COUNTS_PER_INCH;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous
public class Auto extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {
    HalDashboard halDashboard;

    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        FtcValueMenu firstDriveDistanceMenu = new FtcValueMenu("First Drive Distance", null, this, 0, 100, 1, 0, "%.0f in");
        FtcValueMenu secondDriveDistanceMenu = new FtcValueMenu("Second Drive Distance", firstDriveDistanceMenu, this,
                0, 100, 1, 48, "%.0f in");
        firstDriveDistanceMenu.setChildMenu(secondDriveDistanceMenu);
        FtcMenu.walkMenuTree(firstDriveDistanceMenu, this);
        halDashboard.resetTelemetryForOpMode();

        double firstDriveDistance = 48;
        double secondDriveDistance = secondDriveDistanceMenu.getCurrentValue();
        telemetry.log().add("First Drive Distance is %.0f in", firstDriveDistance);
        telemetry.log().add("Second Drive Distance is %.0f in", secondDriveDistance);

        waitForStart();
        //autoFunctions.driveStraightSimple((int) (firstDriveDistance * ENCODER_COUNTS_PER_INCH), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        autoFunctions.shoot();
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraightSimple((int) (secondDriveDistance * ENCODER_COUNTS_PER_INCH), AutoFunctions.DriveStraightDirection.FORWARD, .5);
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
        if (halDashboard == null) {
            halDashboard = new HalDashboard(telemetry);
        }
        halDashboard.resetTelemetryForHalDashboard();
        return halDashboard;
    }
}
