package org.firstinspires.ftc.griffins.Testing;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by David on 12/17/2016.
 */

public class PIDController { //for upcoming comp, just use P and D controllers

    private double kP, kI, kD;
    private double setPoint;
    private double sensorValue;
    private double error;
    private double lastPropTerm;
    private double propTerm;
    private double intTerm;
    private double derTerm;

    private boolean isOnTarget = false;

    private Func<Double> source;
    private DcMotor output; //how you do this

    private boolean isEnabled = false;

    public PIDController(double kP, double kI, double kD, Func<Double> source, DcMotor output){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        //this.setPoint = setPoint;
        this.source = source;
        this.output = output;
    }

    public boolean isOnTarget(){
        if(error==0)
            isOnTarget = true;
        else
            isOnTarget = false;
        return isOnTarget;
    }

    public double getSourceVal(){
        return source.value();
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public double sendPIDOutput(){
        isEnabled = true;

        calculate();

        double control = propTerm+intTerm+derTerm;
        control = Range.clip(control, -1, 1);
        if(output != null)
         output.setPower(control);
        return control;
        //return control; //send to output (motor value = control + currValue)
    }

    public void PIDdisable(){
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

    public void calculate(){
        double intError;


        if (isEnabled){
            sensorValue = source.value();
            error = setPoint - sensorValue;
            propTerm = kP*error;
            intError = kI*propTerm;

            if(intError > 0){
                intTerm += intError;
            }

            if(intError < 0){
                intTerm -= intError;
            }else
                intTerm += intError;

            derTerm = kD*(propTerm - lastPropTerm);

            lastPropTerm = propTerm;

        }
    }


}
