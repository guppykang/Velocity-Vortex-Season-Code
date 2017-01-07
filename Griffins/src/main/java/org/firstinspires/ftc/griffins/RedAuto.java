package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.griffins.MenuPort.FtcChoiceMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;

import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_SERVO;
import static org.firstinspires.ftc.griffins.RobotHardware.LEFT_BUTTON_PUSHER_SENSOR;
import static org.firstinspires.ftc.griffins.RobotHardware.LEFT_COLOR_SENSOR_ADDRESS;
import static org.firstinspires.ftc.griffins.RobotHardware.RIGHT_BUTTON_PUSHER_SENSOR;
import static org.firstinspires.ftc.griffins.RobotHardware.RIGHT_COLOR_SENSOR_ADDRESS;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous
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

        Servo buttonPusherServo = hardwareMap.get(Servo.class, BUTTON_PUSHER_SERVO);
        buttonPusherServo.setDirection(Servo.Direction.FORWARD);
        this.hardware.pushButton(RobotHardware.BeaconState.UNDEFINED_STATE);

        ColorSensor leftButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, LEFT_BUTTON_PUSHER_SENSOR);
        leftButtonPusherColorSensor.setI2cAddress(LEFT_COLOR_SENSOR_ADDRESS);
        leftButtonPusherColorSensor.enableLed(false);

        ColorSensor rightButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, RIGHT_BUTTON_PUSHER_SENSOR);
        rightButtonPusherColorSensor.setI2cAddress(RIGHT_COLOR_SENSOR_ADDRESS);
        rightButtonPusherColorSensor.enableLed(false);

        waitForStart();

        //shoot two particles
        autoFunctions.shoot();
        telemetry.log().add("Finished Shooting");
        telemetry.update();

        //drive straight a little to get off wall in order to turn
        autoFunctions.driveStraight((long)(3/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        telemetry.log().add("Off the Wall");
        telemetry.update();

        //turn so facing toward beacon
        autoFunctions.twoWheelTurnSimple((long)(6.5/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.TurnDirection.LEFT, .25);
        telemetry.log().add("Turned towards beacon");
        telemetry.update();

        //drive toward beacon wall
        autoFunctions.driveStraight((long)(63/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        //turn 180 degrees so beacon pusher is on wall side
        autoFunctions.twoWheelTurnSimple((long)(25/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.TurnDirection.RIGHT, .3);
        telemetry.log().add("Pusher on Beacon side");
        telemetry.update();

        //"parallel parking"
        autoFunctions.curveDriveShort((long)(3/hardware.INCHES_PER_ENCODER_COUNT), (long)(12/hardware.INCHES_PER_ENCODER_COUNT), .1, .9);
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
        telemetry.update();

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