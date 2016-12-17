package org.firstinspires.ftc.griffins.Testing;

import android.hardware.Sensor;

import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 12/17/2016.
 */

public class PIDController { //for upcoming comp, just use P and D controllers

    RobotHardware hardware;

    private double kP, kI, kD;
    private double setPoint;
    private double sensorValue;
    private double error;
    private double output;

    private Sensor sensor;

    private boolean isEnabled = false;

    public PIDController(double kP, double kI, double kD, double setPoint){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setPoint = setPoint;

        hardware = new RobotHardware();
    }

    public void setPID(){
        isEnabled = true;
    }

    public void PIDdisable(){ //check this
        isEnabled = false;
        kI = 0.0;
        kD = 0.0;
    }

    public double getSetPoint(){
        return setPoint;
    }

    public double getError(){
        return error;
    }

    public void calculate(Sensor sensor){
        this.sensor = sensor;
        error = setPoint - sensorValue; //no current sensor value !! use sensor to do so

        while(isEnabled){
            output = kP*error;

        }
    }


}
