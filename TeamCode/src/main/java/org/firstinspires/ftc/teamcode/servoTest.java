package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoTest extends LinearOpMode {
    public Servo servo1;

    public void initialize(){
        servo1=hardwareMap.get(Servo.class,"test1"); //init hw map for following devices

    }
    public void runOpMode(){
        init();
        initialize();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
            servo1.setPosition(gamepad1.left_trigger-gamepad1.right_trigger);
            }
            //we do not care
            telemetry.addData("position",servo1.getPosition());
        }

    }
}
