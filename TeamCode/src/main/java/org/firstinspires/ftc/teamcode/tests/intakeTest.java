package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="INTAKETEST", group="intaketest")

public class intakeTest extends OpMode {
    DcMotor intake;
    public void init(){
        intake=hardwareMap.dcMotor.get("intake");
    }
    public void loop(){
        intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
    }
}
