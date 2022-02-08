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
@TeleOp (name = "liftPid")
public class liftPIDTest extends LinearOpMode {

    public HardwareMap hwMap;
    public hwMecanum robot;
    public lift robotLift;


    public void runOpMode(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        robotLift=new lift(hardwareMap);
        robotLift.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Lift1:",robot.lift1.getCurrentPosition());
            telemetry.addData("error",robotLift.getError());
            telemetry.update();
            if (gamepad1.a){
                robotLift.setHeight(800);
            }
            if (gamepad1.b){
                robotLift.setHeight(350);
            }
            if (gamepad1.x){
                robotLift.resetLift();
            }

            robotLift.updateLift();


            telemetry.addData("Lift1:",robot.lift1.getCurrentPosition());
            telemetry.addData("error",robotLift.getError());
            telemetry.update();

        }

    }
}


