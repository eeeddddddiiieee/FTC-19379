package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.hwMecanum;

public class lift extends hwMecanum{

    public enum liftHeight{
        Low,
        Med,
        High,
    }
    public double targetPosition;

    public static double ki=0;
    public static double kp=.0001;
    public static double kd=.0001;
    public ElapsedTime period = new ElapsedTime();
    public double error1;
    public double integralSum;


    public double lastError;
    public ElevatorFeedforward ff1=new ElevatorFeedforward(.1,.1,.1,.1);
    public void setPosition1(liftHeight height){
        lift1.setPower(ff1.calculate(1,1));
    }


    public lift(HardwareMap hardwareMap){
        super(hardwareMap);
        period = new ElapsedTime();
        period.reset();
        targetPosition=0;
        error1=0;
        integralSum=0;
        lastError=0;

    }

    public void setPosition(liftHeight height){
        double ticks=0;
        switch(height){
            case Low: ticks=0;
            case Med: ticks=800;
            case High: ticks=350;
        }
        setHeight(ticks);

    }

    public void setHeight(double tick){
        targetPosition=tick;

    }
    public boolean isAtTarget(){
        return Math.abs(targetPosition + /*+ because lift is inverted*/ lift1.getCurrentPosition()) < 20;
    }

    public void updateLift(){
        double reference=targetPosition;


        double position;
        while (!isAtTarget()){
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
            lift1.setPower(-output);
            lift2.setPower(-output);
            lastError = error;
            // reset the timer for next time
            period.reset();
            error1=lastError;
        }

        if (targetPosition==0&&liftLimitSwitch.isPressed()){
            resetLift();
            lift1.setPower(0);
            lift2.setPower(0);
            error1=0;

        }
    }


    public void resetLift(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getHeight(){
        return lift1.getCurrentPosition();
    }
    public double getError(){
        return error1;
    }
}
