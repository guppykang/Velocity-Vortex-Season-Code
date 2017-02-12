package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.griffins.RobotHardware;

import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.UNDEFINED;

/**
 * Created by David on 2/11/2017.
 */

@TeleOp(group = "test")
@Disabled
public class IntakeColorSensorTest extends OpMode {

    ModernRoboticsI2cColorSensor colorSensor;
    RobotHardware robotHardware = new RobotHardware();
    RobotHardware.BeaconState alliance = RED;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "intake color");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x32));
        colorSensor.enableLed(false);
        colorSensor.enableLed(true);

        robotHardware.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        double loaderPower;
        double intakePower = gamepad1.left_trigger;

        RobotHardware.BeaconState ball = findColorSensorState(colorSensor);

        if (ball == alliance) {
            loaderPower = 1;
        } else if (ball == UNDEFINED) {
            loaderPower = gamepad1.right_trigger;
        } else {
            loaderPower = -1;
            intakePower = -1;
        }

        robotHardware.getIntake().setPower(intakePower);
        robotHardware.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        robotHardware.setLoaderPower(loaderPower);

        telemetry.addData("color sensor sees", findColorSensorState(colorSensor));
        telemetry.addData("Red, blue, green, alpha", colorSensor.red() + ", " + colorSensor.blue() + ", " + colorSensor.green()
                + ", " + colorSensor.alpha());
    }

    private RobotHardware.BeaconState findColorSensorState(ColorSensor colorSensor) {
        RobotHardware.BeaconState colorState = UNDEFINED;

        if (colorSensor.red() > colorSensor.blue() && colorSensor.alpha() > 10) {
            colorState = RED;
        } else if (colorSensor.alpha() > 10) {
            colorState = BLUE;
        }

        return colorState;
    }
}
