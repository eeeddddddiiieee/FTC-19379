package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SERVOTEST", group="intaketest")

public class servoTest extends OpMode {
    public Servo servo1;
    public CRServo servo2;
    public Servo servo3;
    public Servo servo4;
    public double offset;

    public void init(){
        servo1=hardwareMap.get(Servo.class,"tsepitch"); //init hw map for following devices
        servo3=hardwareMap.get(Servo.class,"tseyaw");
        servo2=hardwareMap.get(CRServo.class,"carousel");
        servo4=hardwareMap.get(Servo.class,"depositExtension");
        offset=0;
        servo1.setPosition(.53);
        servo4.setPosition(.49);
    }

    public void loop(){
        if (gamepad2.right_bumper&&offset<=0){
            offset+=.0005;
        }
        if (gamepad2.left_bumper&&offset>=-.65){
            offset-=.0005;
        }
            servo4.setPosition(Range.clip((gamepad1.right_trigger),.34,.74));

            servo1.setPosition(Range.clip((.48+(-gamepad2.right_stick_y*.15)),.38,.52));
            servo3.setPosition(Range.clip(.975+offset,.35,.98));
            servo2.setPower(gamepad2.left_trigger);
            telemetry.addData("position",servo4.getPosition());
            telemetry.addData("offset:",offset);
            telemetry.update();
    }
}
