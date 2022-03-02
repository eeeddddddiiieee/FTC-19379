package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.robotControls.lift;


@Config
@TeleOp (name = "LIFTPIDTEST")
public class liftPIDTest extends LinearOpMode {

    public HardwareMap hwMap;
    public hwMecanum robot;
    public FtcDashboard d1;
    public Telemetry d1T;
    public lift robotLift;


    public void runOpMode(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        robotLift=new lift(hardwareMap);
        robotLift.init(hardwareMap);
        d1=FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                robotLift.setHeight(900, lift.resetMode.NO);
            }
            if (gamepad1.b){
                robotLift.setHeight(400, lift.resetMode.NO);
            }
            if (gamepad1.x){
                robotLift.setHeight(0, lift.resetMode.YES);
            }

            robotLift.updateLift(robot.lift1.getCurrentPosition());

            telemetry.addData("Lift1:",robot.lift1.getCurrentPosition());
            telemetry.addData("Lift2:",robot.lift1.getCurrentPosition());
            telemetry.addData("error",robotLift.getError());
            telemetry.addData("target",robotLift.targetPosition);
            telemetry.addData("target?",robotLift.isAtTarget());
            telemetry.addData("reset?",robotLift.mode1);
            telemetry.update();

            if (isStopRequested()) return;
        }
    }
}