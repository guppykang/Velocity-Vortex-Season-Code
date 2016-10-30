package org.firstinspires.ftc.griffins;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by David on 1/1/2016.
 * <p/>
 * To control multiple motors at the same time
 * Note that retaining
 */

public class SyncedDcMotors implements DcMotor {

    //constants
    public static final int ALL_SAME = 0;
    public static final int ALTERNATING = 1;

    //variables
    DcMotor[] motors;
    int directionPattern;

    //note it is illegal to pass zero motors names
    public SyncedDcMotors(HardwareMap hardwareMap, DcMotor.Direction direction, int directionPattern, String... motorName) {
        if (motorName.length == 0) {
            throw new IllegalArgumentException("can not take 0 motors");
        }

        this.directionPattern = directionPattern;
        motors = new DcMotor[motorName.length];

        for (int i = 0; i < motorName.length; i++) {
            motors[i] = hardwareMap.dcMotor.get(motorName[i]);
        }

        setDirection(direction);
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        String deviceName = "Synced DC Motors";
        if (motors.length <= 3) {
            deviceName += ":";
            for (DcMotor motor :
                    motors) {
                deviceName += " " + motor.getDeviceName();
            }
        }
        return deviceName;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    public void setDirection(DcMotor.Direction direction) {
        for (int i = 0; i < motors.length; i++) {
            if (directionPattern == ALL_SAME) {
                motors[i].setDirection(direction);
            } else if (directionPattern == ALTERNATING) {
                if (i % 2 == 0) {
                    motors[i].setDirection(direction);
                } else {
                    if (direction == DcMotor.Direction.FORWARD) {
                        motors[i].setDirection(DcMotor.Direction.REVERSE);
                    } else {
                        motors[i].setDirection(DcMotor.Direction.FORWARD);
                    }
                }
            }
        }
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (DcMotor motor : motors) {
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void close() {
        for (DcMotor motor : motors) {
            motor.close();
        }
    }



    @Override
    public Direction getDirection() {
        return motors[0].getDirection();
    }


    @Override
    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return motors[0].getPower();
    }

    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        for (DcMotor motor :
                motors) {
            motor.setMaxSpeed(encoderTicksPerSecond);
        }
    }

    @Override
    public int getMaxSpeed() {
        return motors[0].getMaxSpeed();
    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return -1;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motors[0].getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        for (DcMotor motor : motors) {
            motor.setPowerFloat();
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motors[0].getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    @Override
    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        boolean busy = true;
        for (DcMotor motor : motors) {
            busy = busy && motor.isBusy();
        }
        return busy;
    }

    @Override
    public int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public RunMode getMode() {
        return motors[0].getMode();
    }
}