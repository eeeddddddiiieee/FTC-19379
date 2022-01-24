package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.lift;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class Teleop extends LinearOpMode {
    public lift robotlift;
    public hwMecanum robot;
    ElapsedTime runTime=new ElapsedTime();
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
        Trajectory move1 = robot.trajectoryBuilder(new Pose2d(24, -65,Math.toRadians(270)),true)

                .splineTo(new Vector2d(0,-36),Math.toRadians(315))
                .addTemporalMarker(.5, () -> {
                    // This marker runs two seconds into the trajectory

                    // Run your action in here!


                })
                .build();
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        robotlift=new lift(hardwareMap);
        robotlift.init(hardwareMap);
        driveControls dC=new driveControls();

        waitForStart();

        while (opModeIsActive()) {
            localizer1.update();
            Pose2d currentPose = localizer1.getPoseEstimate();
            Pose2d poseVelocity = localizer1.getPoseVelocity();

            switch (dstate1) {
                case START:
                    robot.bucket.setPosition(hwMecanum.bucketDown);
                    robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                    if (gamepad1.a){
                        dstate1=depositState.PRIME;
                    }
                case PRIME:
                    robot.depositServo.setPosition(hwMecanum.depositClosed);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    robot.intakeServo.setPosition(hwMecanum.intakeUp);
                    robot.intake.setPower(-1);
                    if (gamepad1.x){
                        dstate1=depositState.HIGH;
                    }
                    if (gamepad1.y){
                        dstate1=depositState.MID;
                    }
                case MID:
                    robotlift.setPosition(lift.liftHeight.Med);
                case HIGH:
                case DUMP:
            }

            switch (currentMode){
                case DRIVER:

                    dC.driveController(robot);

                    if (gamepad1.right_stick_button) {
                        localizer1.setPoseEstimate(new Pose2d(24, -65, Math.toRadians(90)));
                        robot.followTrajectory(move1);
                        currentMode=ControlState.AUTO;
                    }
                case AUTO:
                    if (gamepad1.x) {
                        //robot.cancelFollowing();
                        currentMode = ControlState.DRIVER;
                    }
                    if (gamepad1.a){
                        robotlift.setPosition(lift.liftHeight.Med);
                        dstate1=depositState.HIGH;
                        //bucket.setPosition(
                    }
                    if (!robot.isBusy()){
                        currentMode=ControlState.DRIVER;
                    }
                    break;


            }
            //telemetry.addData();
        }
    }
}
