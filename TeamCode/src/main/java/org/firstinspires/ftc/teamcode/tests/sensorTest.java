package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="SENSORTEST", group="intaketest")

public class sensorTest extends LinearOpMode {
    public RevColorSensorV3 sensor1;
    public HardwareMap hwMap;
    public TouchSensor limitSwitch;
    public DigitalChannel greenLED1;
    public DigitalChannel redLED1;

    public void initialize(){
        sensor1=hardwareMap.get(RevColorSensorV3.class,"color1"); //init hw map for following devices
        limitSwitch=hardwareMap.get(TouchSensor.class,"limit");
        greenLED1=hardwareMap.get(DigitalChannel.class,"green1");
        redLED1=hardwareMap.get(DigitalChannel.class,"red1");



    }
    public void runOpMode(){
        initialize();
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();

        while (opModeIsActive()) {
            redLED1.setState(FALSE);
            greenLED1.setState(FALSE);
            if (gamepad1.a) {
                redLED1.setState(TRUE);
            } else {
                redLED1.setState(FALSE);

            }
            if (gamepad1.b) {
                greenLED1.setState(TRUE);
            } else {
                greenLED1.setState(FALSE);
            }
            telemetry.addData("Limit Switch", limitSwitch.getValue());
            telemetry.addData("bucket", sensor1);
            telemetry.addData("Red", sensor1.red());
            telemetry.addData("Green", sensor1.green());
            telemetry.addData("Blue", sensor1.blue());
            telemetry.update();
            if (isStopRequested()){return;}
        }
    }

}
