package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SERVOTEST", group="intaketest")

public class servoTest extends OpMode {
    public Servo servo1;
    public double offset;

    public void init(){
        servo1=hardwareMap.get(Servo.class,"deposit"); //init hw map for following devices
        offset=0;
        servo1.setPosition(.5);
    }

    public void loop(){



            servo1.setPosition(Range.clip((gamepad1.left_trigger),.2,.995));
            telemetry.addData("position",servo1.getPosition());
            telemetry.update();


    }
}
