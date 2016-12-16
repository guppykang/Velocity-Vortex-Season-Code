package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.griffins.RobotHardware;

import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_CENTER_POSITION;
import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_LEFT_POSITION;
import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_RIGHT_POSITION;
import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_SERVO;
import static org.firstinspires.ftc.griffins.RobotHardware.LEFT_BUTTON_PUSHER_SENSOR;
import static org.firstinspires.ftc.griffins.RobotHardware.LEFT_COLOR_SENSOR_ADDRESS;
import static org.firstinspires.ftc.griffins.RobotHardware.RIGHT_BUTTON_PUSHER_SENSOR;
import static org.firstinspires.ftc.griffins.RobotHardware.RIGHT_COLOR_SENSOR_ADDRESS;

/**
 * Created by David on 12/15/2016.
 */

@Autonomous

public class BeaconTest extends LinearOpMode {
    private Servo buttonPusherServo;
    private ColorSensor leftButtonPusherColorSensor;
    private ColorSensor rightButtonPusherColorSensor;
    private RobotHardware.BeaconState alliance = RobotHardware.BeaconState.BLUE_BLUE;

    @Override
    public void runOpMode() throws InterruptedException {
        buttonPusherServo = hardwareMap.get(Servo.class, BUTTON_PUSHER_SERVO);
        buttonPusherServo.setDirection(Servo.Direction.FORWARD);
        this.pushButton(RobotHardware.BeaconState.UNDEFINED_STATE);

        leftButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, LEFT_BUTTON_PUSHER_SENSOR);
        leftButtonPusherColorSensor.setI2cAddress(LEFT_COLOR_SENSOR_ADDRESS);
        leftButtonPusherColorSensor.enableLed(false);

        rightButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, RIGHT_BUTTON_PUSHER_SENSOR);
        rightButtonPusherColorSensor.setI2cAddress(RIGHT_COLOR_SENSOR_ADDRESS);
        rightButtonPusherColorSensor.enableLed(false);

        waitForStart();

        RobotHardware.BeaconState beaconState = findBeaconState();
        while (opModeIsActive() && beaconState == RobotHardware.BeaconState.UNDEFINED_STATE) {
            beaconState = findBeaconState();
            telemetry.addData("Beacon State", "Undefined State, Reading again");
            telemetry.update();
        }
        telemetry.addData("Beacon State", beaconState);
        telemetry.update();
        sleep(1000);
        pushButton(beaconState);
        sleep(5000);
        pushButton(RobotHardware.BeaconState.UNDEFINED_STATE);
        sleep(2000);
    }

    public void pushButton(RobotHardware.BeaconState beaconState) {
        if (alliance == RobotHardware.BeaconState.BLUE_BLUE) {
            if (beaconState == RobotHardware.BeaconState.BLUE_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.RED_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.BLUE_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.RED_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            }
        } else if (alliance == RobotHardware.BeaconState.RED_RED) {
            if (beaconState == RobotHardware.BeaconState.BLUE_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.RED_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.BLUE_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == RobotHardware.BeaconState.RED_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
            }
        }

        if (beaconState == RobotHardware.BeaconState.UNDEFINED_STATE) {
            buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
        }

    }

    /**
     * Will check the color sensors to determine the beacon state
     * If either color sensor's state is UNDEFINED_STATE, then this method will return UNDEFINED_STATE.
     *
     * @return the state of the beacon, as a variable of BeaconState,
     */
    public RobotHardware.BeaconState findBeaconState() {
        RobotHardware.BeaconState beaconState = RobotHardware.BeaconState.UNDEFINED_STATE;
        RobotHardware.BeaconState leftSide = findColorSensorState(leftButtonPusherColorSensor);
        RobotHardware.BeaconState rightSide = findColorSensorState(rightButtonPusherColorSensor);

        if (leftSide == RobotHardware.BeaconState.BLUE_BLUE) {
            if (rightSide == RobotHardware.BeaconState.BLUE_BLUE) {
                beaconState = RobotHardware.BeaconState.BLUE_BLUE;
            } else if (rightSide == RobotHardware.BeaconState.RED_RED) {
                beaconState = RobotHardware.BeaconState.BLUE_RED;
            }
        } else if (leftSide == RobotHardware.BeaconState.RED_RED) {
            if (rightSide == RobotHardware.BeaconState.BLUE_BLUE) {
                beaconState = RobotHardware.BeaconState.RED_BLUE;
            } else if (rightSide == RobotHardware.BeaconState.RED_RED) {
                beaconState = RobotHardware.BeaconState.RED_RED;
            }
        }

        return beaconState;
    }

    /**
     * Checks the state of the color sensor to determine what color is being read
     *
     * @param colorSensor the color sensor that will be checked
     * @return A BeaconState, which will be either RED_RED, BLUE_BLUE, or UNDEFINED_STATE.
     */
    private RobotHardware.BeaconState findColorSensorState(ColorSensor colorSensor) {
        RobotHardware.BeaconState colorState;

        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
            colorState = RobotHardware.BeaconState.RED_RED;
        } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            colorState = RobotHardware.BeaconState.BLUE_BLUE;
        } else {
            colorState = RobotHardware.BeaconState.UNDEFINED_STATE;
        }

        return colorState;
    }
}
