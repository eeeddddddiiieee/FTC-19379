package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.hwMecanum;

public class lift extends hwMecanum{

    public enum liftHeight{
        Low,
        Med,
        High,
    }

    public static double ki;
    public static double kp;
    public static double kd;

    public lift(HardwareMap hardwareMap){
        super(hardwareMap);
    }


    public void setPosition(liftHeight height){
        double ticks=0;
        switch(height){
            case Low: ticks=0;
            case Med: ticks=384.5*8/(1.9685*3.14);
            case High: ticks=384.5*15/(1.9685*3.14);
        }
        setHeight(ticks);

    }

    public void setHeight(double tick){
        double reference = tick;

        double integralSum = 0;

        double lastError = 0;
        double position=lift1.getCurrentPosition();
        period.reset();
        while (position!=reference){
            double error;
            // obtain the encoder position
            position = lift1.getCurrentPosition();
            // calculate the error
            error = reference - position;
            // rate of change of the error
            double derivative = (error - lastError) / period.seconds();
            // sum of all error over time
            integralSum = integralSum + (error * period.seconds());
            double output = Range.clip((kp * error) + (ki * integralSum) + (kd * derivative),-1,1);
            lift1.setPower(output);
            lift2.setPower(-output);
            lastError = error;
            // reset the timer for next time
            period.reset();

        }

    }
    public void resetLift(){
        while (!liftLimitSwitch.isPressed()){
            setHeight(-50);
        }
            lift2.setPower(0);
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getHeight(){
        return lift1.getCurrentPosition();
    }
}
