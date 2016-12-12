package org.firstinspires.ftc.griffins.MenuPort;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 12/2/2016.
 */

@Autonomous(name = "Menu Test", group = "Testing")
@Disabled
public class MenuTest extends OpMode implements FtcMenu.MenuButtonsAndDashboard {
    HalDashboard menuDashboard;
    private FtcValueMenu delayMenu;
    private FtcChoiceMenu strategyMenu;
    private FtcValueMenu driveTimeMenu;
    private FtcValueMenu distanceMenu;
    private FtcValueMenu degreesMenu;
    private FtcChoiceMenu allianceMenu;
    //
    // Menu choices.
    //
    private double delay = 0.0;
    private AutoStrategy autoStrategy = AutoStrategy.DO_NOTHING;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;
    private Alliance alliance = Alliance.RED_ALLIANCE;

    public MenuTest() {
        menuDashboard = new HalDashboard(telemetry);
    }

    @Override
    public void init() {

        telemetry.log().add("Robot Ready");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        FtcMenu.runMenus();
    }

    @Override
    public void start() {
        menuDashboard.resetTelemetryForOpMode();
        //
        // Set choices variables.
        //
        delay = delayMenu.getCurrentValue();
        autoStrategy = (AutoStrategy) strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue();
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
    }

    @Override
    public void loop() {
        telemetry.addData("Delay Selected", delay);
        telemetry.addData("Auto Strategy Selected", autoStrategy);
        telemetry.addData("Drive Time Selected", driveTime);
        telemetry.addData("Drive Distance Selected", driveDistance);
        telemetry.addData("Turn Degrees Selected", turnDegrees);
        telemetry.addData("Alliance Selected", alliance);
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
        menuDashboard.resetTelemetryForHalDashboard();
        return menuDashboard;
    }

    private void prepareMenus() {
        //
        // Create the menus.
        //
        delayMenu = new FtcValueMenu("Delay time:", null, this,
                0.0, 10.0, 1.0, 0.0, "%.0f sec");
        strategyMenu = new FtcChoiceMenu("Auto Strategies:", delayMenu, this);
        driveTimeMenu = new FtcValueMenu("Drive time:", strategyMenu, this,
                0.0, 10.0, 1.0, 4.0, "%.0f sec");
        distanceMenu = new FtcValueMenu("Drive distance:", strategyMenu, this,
                1.0, 8.0, 1.0, 1.0, "%.0f ft");
        degreesMenu = new FtcValueMenu("Turn degrees", strategyMenu, this,
                -360.0, 360.0, 90.0, 360.0, "%.0f deg");
        allianceMenu = new FtcChoiceMenu("Alliance:", strategyMenu, this);

        delayMenu.setChildMenu(strategyMenu);

        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING);
        strategyMenu.addChoice("Timed drive", AutoStrategy.TIMED_DRIVE, driveTimeMenu);
        strategyMenu.addChoice("Drive forward", AutoStrategy.DRIVE_AND_TURN, distanceMenu);
        strategyMenu.addChoice("Follow line", AutoStrategy.FOLLOW_LINE, allianceMenu);
        strategyMenu.addChoice("Seek IR", AutoStrategy.SEEK_IR);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE);

        //
        // Walk the menu tree starting with the delay menu as the root
        // menu and get user choices.
        //
        FtcMenu.setRootMenu(delayMenu);
        //
        // Set choices variables.
        //
        delay = delayMenu.getCurrentValue();
        autoStrategy = (AutoStrategy) strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue();
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
    }   //doMenus

    private enum AutoStrategy {
        DO_NOTHING,
        TIMED_DRIVE,
        DRIVE_AND_TURN,
        FOLLOW_LINE,
        SEEK_IR
    }   //enum AutoStrategy

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}
