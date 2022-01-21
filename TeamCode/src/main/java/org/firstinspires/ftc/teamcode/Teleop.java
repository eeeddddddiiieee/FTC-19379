package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.lift;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class Teleop extends LinearOpMode {
    public lift robotlift;
    public hwMecanum robot;
    public enum depositState{
        START,
        PRIME,
        MID,
        HIGH,
        DUMP
    }
    public enum ControlState{
        DRIVER,
        AUTO
    }
    ControlState currentMode= ControlState.DRIVER;


    public void runOpMode(){
        StandardTrackingWheelLocalizer localizer1 = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer1.setPoseEstimate(new Pose2d(0,0,0));
        depositState dstate1=depositState.START;

        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        robotlift=new lift(hardwareMap);
        robotlift.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            localizer1.update();
            Pose2d currentPose = localizer1.getPoseEstimate();
            Pose2d poseVelocity = localizer1.getPoseVelocity();
            switch (currentMode){
                case DRIVER:
                    switch (dstate1) {
                        case START:
                        case MID:
                        case HIGH:
                        case DUMP:
                    }

                    if (gamepad1.a) {
                        localizer1.setPoseEstimate(new Pose2d(24, 7, Math.toRadians(90)));
                    }
                case AUTO:


            }
        }




    }
}
