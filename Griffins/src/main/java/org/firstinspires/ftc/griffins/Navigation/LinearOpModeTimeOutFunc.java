package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by David on 12/20/2016.
 */

public class LinearOpModeTimeOutFunc implements Func<Boolean> {
    LinearOpMode opMode;
    ElapsedTime timeOut;
    double timeOutLength;

    public LinearOpModeTimeOutFunc(LinearOpMode opMode, double timeOutLengthSeconds) {
        this.opMode = opMode;
        this.timeOutLength = timeOutLengthSeconds;
        timeOut = new ElapsedTime();
    }

    @Override
    public Boolean value() {
        return opMode.opModeIsActive() && (timeOut.seconds() < timeOutLength);
    }
}
