package org.firstinspires.ftc.griffins;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 10/29/2016.
 * Stores the hardware for the robot.
 */
public class RobotHardware {

    // Hardware map name constants
    public static final String LEFT_DRIVE_ONE = "left1";
    public static final String LEFT_DRIVE_TWO = "left2";
    public static final String RIGHT_DRIVE_ONE = "right1";
    public static final String RIGHT_DRIVE_TWO = "right2";
    public static final String SHOOTER_MOTOR_LEFT = "shooterL";
    public static final String SHOOTER_MOTOR_RIGHT = "shooterR";
    public static final String INTAKE_MOTOR = "intake";
    public static final String TURRET_ROTATION_MOTOR = "turret";
    public static final String LEFT_TURRET_GUIDE_SERVO = "turretL";
    public static final String RIGHT_TURRET_GUIDE_SERVO = "turretR";
    public static final String BUTTON_PUSHER_SERVO = "button";
    public static final String LOADER_SERVO_ONE = "loader";
    public static final String LOADER_SERVO_TWO = "loader2";
    public static final String TURRET_GYRO = "gyro";
    public static final String LEFT_BUTTON_PUSHER_SENSOR = "colorL";
    public static final String RIGHT_BUTTON_PUSHER_SENSOR = "colorR";
    public static final String BEACON_DISTANCE_SENSOR = "distance";
    public static final String LOADER_SWITCH = "loader switch";
    public static final String BNO055_SENSOR = "bno055";

    // The constants for motor encoders
    public static final int NEVEREST_ENCODER_COUNT_PER_ROTATION = 28;
    public static final int NEVEREST_40_ENCODER_COUNTS_PER_ROTATION = NEVEREST_ENCODER_COUNT_PER_ROTATION * 40;
    // The addresses for the color sensors, since we are using two, the default will be changed
    public static final I2cAddr LEFT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x38);
    public static final I2cAddr RIGHT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x3C);
    // The constants for the button pusher positions
    public static final double BUTTON_PUSHER_CENTER_POSITION = 107 / 255.0;
    public static final double BUTTON_PUSHER_RATIO = 2 / 3.0;
    public static final double BUTTON_PUSHER_LEFT_FULL_EXTENSION = 32 / 255.0;
    public static final double BUTTON_PUSHER_RIGHT_FULL_EXTENSION = 182 / 255.0;
    public static final double BUTTON_PUSHER_LEFT_POSITION = (BUTTON_PUSHER_LEFT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * BUTTON_PUSHER_RATIO + BUTTON_PUSHER_CENTER_POSITION;
    public static final double BUTTON_PUSHER_RIGHT_POSITION = (BUTTON_PUSHER_RIGHT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * BUTTON_PUSHER_RATIO + BUTTON_PUSHER_CENTER_POSITION;
    // The constants for the loader speeds
    public static final double LOADER_ZERO_POWER = 0;
    public static final double LOADER_FULL_REVERSE_POWER = -2 / 3.0;
    public static final double LOADER_FULL_FORWARD_POWER = 2 / 3.0;
    // The constants for shooting speeds
    public static final double SHOOTER_SPEED = 0.9;
    // The constants for driving
    public static final double INCHES_PER_ENCODER_COUNT = (2 * Math.PI) / (NEVEREST_ENCODER_COUNT_PER_ROTATION * 10);  // (wheel diameter * pi) / (encoder counts per motor rotation * gear ratio)
    public static final double ENCODER_COUNTS_PER_INCH = 1 / INCHES_PER_ENCODER_COUNT; // inverse of above INCHES_PER_ENCODER_COUNT
    //turret encoder limits
    public static final double ENCODER_COUNTS_PER_TURRET_REVOLUTION = NEVEREST_40_ENCODER_COUNTS_PER_ROTATION * 3;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREE = ENCODER_COUNTS_PER_TURRET_REVOLUTION / 360;
    public static final int TURRET_ENCODER_COUNT_REVOLUTION_LIMIT = (int) (ENCODER_COUNTS_PER_TURRET_DEGREE * 200);
    // The Vuforia License Key
    public static final String VUFORIA_LICENSE_KEY = "AT3JPAj/////AAAAGdMIFxYU7UYXs7ZUAq3xlpUJDbeYIAIe69usc" +
                                                     "Pw2c6g2kjczfs8x9A1YX2Mi3SLEFsx0JM1x9Lm733yP8I8HxGqUGJ" +
                                                     "r2HDUcUheHtJZ3itjlnCaAZWYqeV+RB4bU8t3BW3pnecmQ9BmjcUO" +
                                                     "aO32ENJIc0PRvpzPFNTtKB6HHJwNhPoMZYonmVEVeCwsuSfIhyzl1" +
                                                     "KWHU8GVzgdz3NRBs0O7Dedd+cECw9dmXX0TutXkuMr9ykOstrDXM6" +
                                                     "1D1Hb2DuY+4LKERkLFwUm/TDv5+zR7A4eDoE92nmEIpVdSfR7kNYG" +
                                                     "QGeDbWK7/oHGjwVYOZvEvmTW9dMBDQNiCCeWCag6o4odFTMo5Tc8U6+grD2qVR";
    private BeaconState alliance;
    //motor variables
    private SyncedDcMotors leftDrive;
    private SyncedDcMotors rightDrive;
    private SyncedDcMotors shooter;
    private DcMotor intake;
    private DcMotor turretRotation;

    //servo variables
    private Servo leftTurretGuide;
    private Servo rightTurretGuide;
    private Servo buttonPusherServo;
    private Servo loaderServoOne;
    //private CRServo loaderServoTwo;

    //sensor variables
    private ModernRoboticsI2cGyro turretGyro;
    private ColorSensor leftButtonPusherColorSensor;
    private ColorSensor rightButtonPusherColorSensor;
    private ModernRoboticsAnalogOpticalDistanceSensor beaconDistanceSensor;
    private DigitalChannel loaderParticleLimitSwitch;
    private BNO055IMU robotTracker;

    private double turretHeadingTarget;

    public RobotHardware() {
        alliance = BeaconState.BLUE_BLUE;
    }

    public void initialize(HardwareMap hardwareMap) {
        leftDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.REVERSE, SyncedDcMotors.ALL_SAME, LEFT_DRIVE_ONE, LEFT_DRIVE_TWO);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.FORWARD, SyncedDcMotors.ALL_SAME, RIGHT_DRIVE_ONE, RIGHT_DRIVE_TWO);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.REVERSE, SyncedDcMotors.ALTERNATING, SHOOTER_MOTOR_LEFT, SHOOTER_MOTOR_RIGHT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretRotation = hardwareMap.get(DcMotor.class, TURRET_ROTATION_MOTOR);
        turretRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buttonPusherServo = hardwareMap.get(Servo.class, BUTTON_PUSHER_SERVO);
        buttonPusherServo.setDirection(Servo.Direction.FORWARD);
        this.pushButton(BeaconState.UNDEFINED_STATE);

        leftTurretGuide = hardwareMap.get(Servo.class, LEFT_TURRET_GUIDE_SERVO);
        leftTurretGuide.setDirection(Servo.Direction.FORWARD);

        rightTurretGuide = hardwareMap.get(Servo.class, RIGHT_TURRET_GUIDE_SERVO);
        rightTurretGuide.setDirection(Servo.Direction.REVERSE);

        setTurretGuidePosition(0);

        loaderServoOne = hardwareMap.get(Servo.class, LOADER_SERVO_ONE);
        loaderServoOne.setDirection(Servo.Direction.REVERSE);

        /*loaderServoTwo = hardwareMap.get(CRServo.class, LOADER_SERVO_TWO);
        loaderServoTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        loaderServoTwo.setPower(LOADER_ZERO_POWER);*/

        this.setLoaderPower(0);

        turretGyro = (ModernRoboticsI2cGyro) hardwareMap.get(GyroSensor.class, TURRET_GYRO);
        turretGyro.calibrate();  //look at z axis scaling coefficient when available
        turretGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN); // verify that the angles have the correct sign

        /*leftButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, LEFT_BUTTON_PUSHER_SENSOR);
        leftButtonPusherColorSensor.setI2cAddress(LEFT_COLOR_SENSOR_ADDRESS);
        leftButtonPusherColorSensor.enableLed(false);

        rightButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, RIGHT_BUTTON_PUSHER_SENSOR);
        rightButtonPusherColorSensor.setI2cAddress(RIGHT_COLOR_SENSOR_ADDRESS);
        rightButtonPusherColorSensor.enableLed(false);

        beaconDistanceSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, BEACON_DISTANCE_SENSOR);
        beaconDistanceSensor.enableLed(true);


        loaderParticleLimitSwitch = hardwareMap.get(DigitalChannel.class, LOADER_SWITCH);
        loaderParticleLimitSwitch.setMode(DigitalChannelController.Mode.INPUT);

        robotTracker = hardwareMap.get(BNO055IMU.class, BNO055_SENSOR);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelerationIntegrationAlgorithm = new GriffinAccelerationIntegratorLowPass();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = false;
        robotTracker.initialize(parameters);
        robotTracker.startAccelerationIntegration(new Position(), new Velocity(), 10);*/
    }

    public SyncedDcMotors getLeftDrive() {
        return leftDrive;
    }

    public SyncedDcMotors getRightDrive() {
        return rightDrive;
    }

    public SyncedDcMotors getShooter() {
        return shooter;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getTurretRotation() {
        return turretRotation;
    }

    public void setTurretRotation(double joystickInput, boolean trackingOn) {
        if (this.getTurretGyro().isCalibrating()) {
            trackingOn = false;
        }

        double turretSpeed = 0;
        double turretError = 0;

        if (trackingOn) {
            turretHeadingTarget += joystickInput;
            turretError = -((int) turretHeadingTarget - turretGyro.getIntegratedZValue());
            turretSpeed = turretError / 100;
            turretSpeed = Range.clip(turretSpeed, -.5, .5);
        } else {
            turretSpeed = -joystickInput;
            turretHeadingTarget = turretGyro.getIntegratedZValue();
        }

        /*if (turretRotation.getCurrentPosition() > RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, -1, 0);
            if (turretError > 10) {
                turretHeadingTarget += 360;
                //turretSpeed = -1;
            }

        } else if (turretRotation.getCurrentPosition() < -RobotHardware.TURRET_ENCODER_COUNT_REVOLUTION_LIMIT) {
            turretSpeed = Range.clip(turretSpeed, 0, 1);
            if (turretError < -10) {
                turretHeadingTarget -= 360;
                // turretSpeed = 1;
            }
        }*/

        turretRotation.setPower(turretSpeed);

       /* telemetry.addData("Turret Heading Current|Target", turretGyro.getIntegratedZValue() + "|" + turretHeadingTarget);
        telemetry.addData("Turret Error", turretError);
        telemetry.addData("Turret Speed", turretSpeed);
        telemetry.addData("Turret Counts", turretRotation.getCurrentPosition());*/
    }

    public BNO055IMU getRobotTracker() {
        return robotTracker;
    }

    public ModernRoboticsI2cGyro getTurretGyro() {
        return turretGyro;
    }

    public ColorSensor getLeftButtonPusherColorSensor() {
        return leftButtonPusherColorSensor;
    }

    public ColorSensor getRightButtonPusherColorSensor() {
        return rightButtonPusherColorSensor;
    }

    public DigitalChannel getLoaderParticleLimitSwitch() {
        return loaderParticleLimitSwitch;
    }

    public void setTurretGuidePosition(double position) {
        // TODO: 10/29/2016 create code for turret position
    }

    public void pushButton(BeaconState beaconState) {
        if (alliance == BeaconState.BLUE_BLUE) {
            if (beaconState == BeaconState.BLUE_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
            } else if (beaconState == BeaconState.RED_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == BeaconState.BLUE_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
            } else if (beaconState == BeaconState.RED_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            }
        } else if (alliance == BeaconState.RED_RED) {
            if (beaconState == BeaconState.BLUE_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == BeaconState.RED_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
            } else if (beaconState == BeaconState.BLUE_BLUE) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
            } else if (beaconState == BeaconState.RED_RED) {
                buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
            }
        }

        if (beaconState == BeaconState.UNDEFINED_STATE) {
            buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
        }

    }

    /**
     * Will check the color sensors to determine the beacon state
     * If either color sensor's state is UNDEFINED_STATE, then this method will return UNDEFINED_STATE.
     *
     * @return the state of the beacon, as a variable of BeaconState,
     */
    public BeaconState findBeaconState() {
        BeaconState beaconState = BeaconState.UNDEFINED_STATE;
        BeaconState leftSide = findColorSensorState(leftButtonPusherColorSensor);
        BeaconState rightSide = findColorSensorState(rightButtonPusherColorSensor);

        if (leftSide == BeaconState.BLUE_BLUE) {
            if (rightSide == BeaconState.BLUE_BLUE) {
                beaconState = BeaconState.BLUE_BLUE;
            } else if (rightSide == BeaconState.RED_RED) {
                beaconState = BeaconState.BLUE_RED;
            }
        } else if (leftSide == BeaconState.RED_RED) {
            if (rightSide == BeaconState.BLUE_BLUE) {
                beaconState = BeaconState.RED_BLUE;
            } else if (rightSide == BeaconState.RED_RED) {
                beaconState = BeaconState.RED_RED;
            }
        }

        return beaconState;
    }

    /**
     * Checks the state of the color sensor to determine what color is being read
     * @param colorSensor the color sensor that will be checked
     * @return A BeaconState, which will be either RED_RED, BLUE_BLUE, or UNDEFINED_STATE.
     */
    private BeaconState findColorSensorState(ColorSensor colorSensor) {
        BeaconState colorState;

        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
            colorState = BeaconState.RED_RED;
        } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            colorState = BeaconState.BLUE_BLUE;
        } else {
            colorState = BeaconState.UNDEFINED_STATE;
        }

        return colorState;
    }

    public void setLoaderPower(double power) {
        power = Range.clip(power, -1, 1);
        power = Range.scale(power, -1, 1, 0, 1);
        loaderServoOne.setPosition(power);
        //loaderServoTwo.setPower(power);
    }

    public void startTurretTracking() {
        turretGyro.resetZAxisIntegrator();
        turretHeadingTarget = 0;
    }

    public enum BeaconState {
        BLUE_BLUE,
        BLUE_RED,
        RED_BLUE,
        RED_RED,
        UNDEFINED_STATE
    }
}
