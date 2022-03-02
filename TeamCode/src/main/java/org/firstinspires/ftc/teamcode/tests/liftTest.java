package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//@TeleOp (name = "liftTest")
public class liftTest extends LinearOpMode {

    public HardwareMap hwMap;
    public DcMotorEx lift1;
    public DcMotorEx lift2;

    public void initialize(HardwareMap aHwMap){
        hwMap=aHwMap;
        lift1=hardwareMap.get(DcMotorEx.class,"lift1"); //init hw map for following devices
        lift2=hardwareMap.get(DcMotorEx.class,"lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void runOpMode(){
        initialize(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            lift1.setPower(gamepad1.right_stick_y);
            lift2.setPower(gamepad1.right_stick_y);
            telemetry.addData("Lift1:",lift1.getCurrentPosition());
            telemetry.addData("Lift2:",lift2.getCurrentPosition());
            telemetry.update();
            if (isStopRequested()){
                return;
            }


        }

    }
}


