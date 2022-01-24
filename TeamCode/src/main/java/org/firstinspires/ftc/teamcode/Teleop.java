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

    public hwMecanum robot;
    public depositStateMachine deposit1;
    ElapsedTime runTime=new ElapsedTime();

    public enum ControlState{
        DRIVER,
        AUTO
    }
    ControlState currentMode= ControlState.DRIVER;
    //public boolean intakeMode=true;


    public void runOpMode(){
        StandardTrackingWheelLocalizer localizer1 = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer1.setPoseEstimate(new Pose2d(0,0,0));

        Trajectory move1 = robot.trajectoryBuilder(new Pose2d(24, -65,Math.toRadians(270)),true)

                .splineTo(new Vector2d(0,-36),Math.toRadians(315))
                .addTemporalMarker(.5, () -> {
                    // This marker runs two seconds into the trajectory

                    // Run your action in here!
                })
                .build();
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        deposit1=new depositStateMachine();
        deposit1.initDeposit();
        implementController ic1=new implementController();
        ic1.initialize(robot);

        driveControls dC=new driveControls();

        waitForStart();

        while (opModeIsActive()) {
            if (getRuntime()>95000){
                gamepad1.rumble(500);
                gamepad2.rumble(500);

            }

            localizer1.update();
            Pose2d currentPose = localizer1.getPoseEstimate();
            Pose2d poseVelocity = localizer1.getPoseVelocity();
            deposit1.deposit(robot);

            switch (currentMode){
                case DRIVER:
                    ic1.runImplementController(robot);
                    dC.driveController(robot);

                    if (gamepad1.left_stick_button) {
                        localizer1.setPoseEstimate(new Pose2d(24, -65, Math.toRadians(90)));
                        robot.followTrajectory(move1);
                        currentMode=ControlState.AUTO;
                    }

                    if (deposit1.intakeMode) {
                        robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    }
                case AUTO:
                    if (gamepad1.x) {
                        robot.cancelFollowing();
                        currentMode = ControlState.DRIVER;
                    }

                    if (!robot.isBusy()){
                        currentMode=ControlState.DRIVER;
                    }
                    break;


            }
            telemetry.addData("POSITION:",currentPose.getX()+","+currentPose.getY());
            telemetry.addData("HEADING:",currentPose.getHeading());
            telemetry.addData("DEPOSIT:",deposit1.getDepositState());
            telemetry.addData("RUNTIME:",getRuntime());
            telemetry.addData("CARGO","YES");
            telemetry.update();
        }
    }
}
