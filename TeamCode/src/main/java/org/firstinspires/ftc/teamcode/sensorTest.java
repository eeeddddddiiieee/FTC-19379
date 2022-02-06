package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class sensorTest extends LinearOpMode {
    public RevColorSensorV3 sensor1;
    public HardwareMap hwMap=hardwareMap;
    public TouchSensor limitSwitch;
    public DigitalChannel greenLED1;
    public DigitalChannel redLED1;

    public void initialize(){
        sensor1=hwMap.get(RevColorSensorV3.class,"intakeSensor"); //init hw map for following devices
        limitSwitch=hwMap.get(TouchSensor.class,"limitSwitch");
        greenLED1=hwMap.get(DigitalChannel.class,"greenled1");
        redLED1=hwMap.get(DigitalChannel.class,"redled1");


    }
    public void runOpMode(){
        if (gamepad1.left_bumper){
            redLED1.setState(TRUE);
        }
        else {
            redLED1.setState(FALSE);
        }
        if (gamepad1.right_bumper){
            greenLED1.setState(TRUE);
        }
        else {
            greenLED1.setState(FALSE);
        }
        telemetry.addData("Limit Switch",limitSwitch);
        telemetry.addData("bucket",sensor1);
        telemetry.addData("Red",sensor1.red());
        telemetry.addData("Green",sensor1.green());
        telemetry.addData("Blue",sensor1.blue());
        telemetry.update();
    }

}