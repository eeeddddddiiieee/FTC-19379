package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class implementController extends LinearOpMode {
    public double clawPosition;
    public double clawOffset;
    public double clawSpeed=.1;
    public double TEposition;
    public static final double TEUp=.5;
    public static final double TEMax=.9;
    public static final double TEMin=.1;
    public void runOpMode(){

    }
    public void initialize(hwMecanum robot){
        clawPosition=0;
        robot.claw.setPosition(clawPosition+clawOffset);
        clawOffset=0;
        TEposition=TEUp;
    }
    public void runImplementController(hwMecanum robot){
        //team element claw
        robot.teamElementArm.setPower(gamepad2.left_stick_y*.4);
        robot.carousel.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

        if (gamepad2.left_bumper){
            clawOffset+=clawSpeed;
        }
        if (gamepad2.right_bumper){
            clawOffset-=clawSpeed;
        }
        //robot.claw.setPosition(Range.clip(clawPosition+clawOffset,0,1));
        double armPositionRadians=((robot.teamElementArm.getCurrentPosition()/1992.6)*2*Math.PI)-(Math.PI/2);

        /*
        if (gamepad2.y){
            double servoPositionRadians=(robot.TE.getPosition()*2*Math.PI)+Math.PI/6; //fix, should be relative to the arm, top position should be 1

            //servo+arm=270 degrees
            double servoTargetPositionRadians=Math.PI*1.5-armPositionRadians;
            double servoTargetPosition=(servoTargetPositionRadians-Math.PI/6)/(2*Math.PI);


            TEposition=Range.clip((servoTargetPosition),0,1);
        }*/
        /*
        if (gamepad2.start){
            TEposition=TEUp;
        }
        robot.TE.setPosition(Range.clip(TEposition, TEMin, TEMax));
        */

    }
}
