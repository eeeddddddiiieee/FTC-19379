package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.hwMecanum;

@Config
public class lift extends hwMecanum{

    public enum liftHeight{
        Low,
        Med,
        High,
    }
    public double targetPosition;

    public static double ki=.00001;
    public static double kp=.003;
    public static double kd=.0000;
    public ElapsedTime period = new ElapsedTime();
    public double error1;
    public double integralSum;


    public double lastError;
    public ElevatorFeedforward ff1=new ElevatorFeedforward(.1,.1,.1,.1);
    public void setPosition1(liftHeight height){
        lift1.setPower(ff1.calculate(1,1));
    }

    public enum resetMode{
        YES,
        NO
    }

    public resetMode mode1;

    public lift(HardwareMap hardwareMap){
        super(hardwareMap);
        period = new ElapsedTime();
        period.reset();
        targetPosition=0;
        mode1=resetMode.NO;
        error1=0;
        integralSum=0;
        lastError=0;

    }

    public void setPosition(liftHeight height){
        double ticks=0;
        switch(height){
            case Low: ticks=0;
            case Med: ticks=900;
            case High: ticks=400;
        }
        setHeight(ticks,resetMode.NO);

    }

    public void setHeight(double tick,resetMode b){
        targetPosition=tick;
        mode1=b;


    }
    public boolean isAtTarget(){
        return Math.abs(targetPosition - lift1.getCurrentPosition()) < 10;
    }

    public void updateLift(double currentPosition){
        double reference=targetPosition;

        double position;

        if (mode1==resetMode.YES) {
            if (!liftLimitSwitch.isPressed()) {
                /*
                double error;
                // obtain the encoder position
                position = currentPosition;
                // calculate the error
                error = reference - position;
                // rate of change of the error
                double derivative = (error - lastError) / period.seconds();
                // sum of all error over time
                integralSum = integralSum + (error * period.seconds());
                double output = Range.clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);
                lift1.setPower(output);
                lift2.setPower(output);
                lastError = error;
                // reset the timer for next time
                period.reset();
                error1 = lastError;

                 */
                if (lift1.getCurrentPosition()>80){
                lift1.setPower(-1);
                lift2.setPower(-1);
                }
                else {
                    lift1.setPower(-.5);
                    lift2.setPower(-.5);
                }

            }
            else {
                resetLift();
                lift1.setPower(0);
                lift2.setPower(0);
                targetPosition = 0;
                error1 = 0;
            }

        }

        if (!isAtTarget()&&mode1==resetMode.NO) {
            double error;
            // obtain the encoder position
            position = currentPosition;
            // calculate the error
            error = reference - position;
            // rate of change of the error
            double derivative = (error - lastError) / period.seconds();
            // sum of all error over time
            integralSum = integralSum + (error * period.seconds());
            double output = Range.clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);
            lift1.setPower(output);
            lift2.setPower(output);
            lastError = error;
            // reset the timer for next time
            period.reset();
            error1 = lastError;
        }


    }


    public void resetLift(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public double getHeight(){
        return lift1.getCurrentPosition();
    }
    public double getError(){
        return error1;
    }
}
