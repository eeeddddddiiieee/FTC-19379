package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.robotControls.depositStateMachine;
import org.firstinspires.ftc.teamcode.robotControls.driveControls;
import org.firstinspires.ftc.teamcode.robotControls.implementController;

@TeleOp(name="TELEOP", group="hwMecanum")

public class Teleop extends LinearOpMode {

    ElapsedTime runTime=new ElapsedTime();

    public enum ControlState{
        DRIVER,
        AUTO
    }
    ControlState currentMode= ControlState.DRIVER;
    //public boolean intakeMode=true;


    public void runOpMode() throws InterruptedException{
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        double shift;
        double reverse;
        double x1;
        double y1;
        double yaw;

        depositStateMachine deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        implementController ic1=new implementController();
        ic1.initialize(robot);

        driveControls dC=new driveControls();

        robot.isCargo=false;
        //localizer1.setPoseEstimate(new Pose2d(0,0,0));

        Trajectory move1 = robot.trajectoryBuilder(new Pose2d(24, -65,Math.toRadians(270)),true)

                .splineTo(new Vector2d(0,-36),Math.toRadians(315))
                .addTemporalMarker(.5, () -> {
                    // This marker runs two seconds into the trajectory

                    // Run your action in here!
                })
                .build();

        waitForStart();

        while (opModeIsActive()) {
            //localizer1.update();
            //Pose2d currentPose = localizer1.getPoseEstimate();
            //Pose2d poseVelocity = localizer1.getPoseVelocity();
            deposit1.deposit(robot,gamepad1);
            deposit1.updatePID(robot);
            ic1.runImplementController(robot,gamepad1,gamepad2);
            dC.driveController(robot,gamepad1);

            if (robot.intakeMode==false) {
                robot.intake.setPower(deposit1.intakePower);
            }
            else if ( robot.intakeMode&&!gamepad1.dpad_down&&!gamepad1.dpad_up&&!robot.toggle) {
                robot.intake.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*.65);
            }
            /*
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
                        //robot.cancelFollowing();
                        localizer1.setPoseEstimate(new Pose2d(0, -36, Math.toRadians(315)));

                        robot.followTrajectory(null);
                        robot.q1.setPower(0);
                        robot.q2.setPower(0);
                        robot.q3.setPower(0);
                        robot.q4.setPower(0);

                        currentMode = ControlState.DRIVER;
                    }

                    if (!robot.isBusy()){
                        currentMode=ControlState.DRIVER;
                    }
                    break;
            }
            */

            //telemetry.addData("POSITION:",currentPose.getX()+","+currentPose.getY());
            //telemetry.addData("HEADING:",currentPose.getHeading());
            //telemetry.addData("DEPOSIT:",deposit1.getDepositState());
            if (getRuntime()==80){
                gamepad1.rumble(.5,.5,1500);
                gamepad2.rumble(.5,.5,1500);

            }

            if (getRuntime()==90){
                gamepad1.rumble(1,1,200);
                gamepad2.rumble(1,1,200);
            }

            if (robot.bucketSensor.getDistance(DistanceUnit.MM)<20){
                robot.isCargo=TRUE;
            }
            else {
                robot.isCargo=FALSE;
            }

            telemetry.addData("RUNTIME:",getRuntime());
            telemetry.addData("CARGO:", robot.bucketSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("lift",deposit1.robotlift.getHeight());
            telemetry.addData("lift",deposit1.robotlift.targetPosition);

            telemetry.update();
        }
    }
}
