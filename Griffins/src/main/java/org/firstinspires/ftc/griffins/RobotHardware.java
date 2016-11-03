package org.firstinspires.ftc.griffins;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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
    public static final String LOADER_SERVO_ONE = "loader1";
    public static final String LOADER_SERVO_TWO = "loader2";
    public static final String TURRET_GYRO = "gyro";
    public static final String LEFT_BUTTON_PUSHER_SENSOR = "colorL";
    public static final String RIGHT_BUTTON_PUSHER_SENSOR = "colorR";
    public static final String BEACON_DISTANCE_SENSOR = "distance";
    public static final String LOADER_SWITCH = "loader switch";
    public static final String BNO055_SENSOR = "bno055";

    // The addresses for the color sensors, since we are using two, the default will be changed
    public static final I2cAddr LEFT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x3C);
    public static final I2cAddr RIGHT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x38);

    //motor variables
    private SyncedDcMotors leftDrive;
    private SyncedDcMotors rightDrive;
    private SyncedDcMotors shooter;
    private DcMotor intake;
    private DcMotor turretRotation;

    //servo variables
    private Servo leftTurretGuide;
    private Servo rightTurretGuide;
    private CRServo buttonPusherServo;
    private CRServo loaderServoOne;
    private CRServo loaderServoTwo;

    //sensor variables
    private ModernRoboticsI2cGyro turretGyro;
    private ColorSensor leftButtonPusherColorSensor;
    private ColorSensor rightButtonPusherColorSensor;
    private ModernRoboticsAnalogOpticalDistanceSensor beaconDistanceSensor;
    private DigitalChannel loaderParticleLimitSwitch;
    private BNO055IMU robotTracker;

    public RobotHardware() {
    }

    public void initialize(HardwareMap hardwareMap) {
        leftDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.FORWARD, SyncedDcMotors.ALL_SAME, LEFT_DRIVE_ONE, LEFT_DRIVE_TWO);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.FORWARD, SyncedDcMotors.ALL_SAME, RIGHT_DRIVE_ONE, RIGHT_DRIVE_TWO);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.FORWARD, SyncedDcMotors.ALTERNATING, SHOOTER_MOTOR_LEFT, SHOOTER_MOTOR_RIGHT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretRotation = hardwareMap.get(DcMotor.class, TURRET_ROTATION_MOTOR);
        turretRotation.setDirection(DcMotorSimple.Direction.FORWARD);
        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurretGuide = hardwareMap.get(Servo.class, LEFT_TURRET_GUIDE_SERVO);
        leftTurretGuide.setDirection(Servo.Direction.FORWARD);

        rightTurretGuide = hardwareMap.get(Servo.class, RIGHT_TURRET_GUIDE_SERVO);
        rightTurretGuide.setDirection(Servo.Direction.REVERSE);

        setTurretGuidePosition(0);

        double zeroPower = 0; // TODO: 10/29/2016 check that this is no power

        buttonPusherServo = hardwareMap.get(CRServo.class, BUTTON_PUSHER_SERVO);
        buttonPusherServo.setDirection(DcMotorSimple.Direction.FORWARD);
        buttonPusherServo.setPower(zeroPower);

        loaderServoOne = hardwareMap.get(CRServo.class, LOADER_SERVO_ONE);
        loaderServoOne.setDirection(DcMotorSimple.Direction.FORWARD);
        loaderServoOne.setPower(zeroPower);

        loaderServoTwo = hardwareMap.get(CRServo.class, LOADER_SERVO_TWO);
        loaderServoTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        loaderServoTwo.setPower(zeroPower);

        turretGyro = (ModernRoboticsI2cGyro) hardwareMap.get(GyroSensor.class, TURRET_GYRO);
        turretGyro.calibrate();  //look at z axis scaling coefficient when available
        turretGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN); // verify that the angles have the correct sign

        leftButtonPusherColorSensor = hardwareMap.get(ColorSensor.class, LEFT_BUTTON_PUSHER_SENSOR);
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
        robotTracker.startAccelerationIntegration(new Position(), new Velocity(), 10);
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

    public void pushButton(/*parameters needed*/) {
        // TODO: 10/29/2016 create code to push button
    }
}
