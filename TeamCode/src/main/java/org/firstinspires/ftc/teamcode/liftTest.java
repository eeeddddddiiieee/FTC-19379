package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class liftTest extends LinearOpMode {

    public HardwareMap hwMap=hardwareMap;
    public DcMotorEx lift1;
    public DcMotorEx lift2;
    public void initialize(){
        lift1=hwMap.get(DcMotorEx.class,"lift1"); //init hw map for following devices
        lift2=hwMap.get(DcMotorEx.class,"lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void runOpMode(){
        init();
        initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Lift1:",lift1.getCurrentPosition());
            telemetry.addData("Lift2:",lift2.getCurrentPosition());
            telemetry.update();

        }

    }
}


