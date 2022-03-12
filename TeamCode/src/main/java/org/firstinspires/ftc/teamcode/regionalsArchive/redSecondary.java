package org.firstinspires.ftc.teamcode.regionalsArchive;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.robotControls.depositStateMachine;
import org.firstinspires.ftc.teamcode.te.vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class redSecondary extends LinearOpMode {
    public hwMecanum robot;
    public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public depositStateMachine deposit1;
    public int signal;
    public double iPower;

    public enum trajState{
        MOVE1,
        MOVE2,
        SHIPPING,
        IDLE
    }
    public double xPo;
    public double yPo;
    public vision.barcodePosition b1;
    public trajState tState1;
    public Pose2d startPose=new Pose2d(-6,-65,Math.toRadians(270));

    public void initialize(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        vision1=new vision();
        vision1.initVision(hardwareMap);
        tState1=trajState.IDLE;
        xPo=44;
        yPo=-65;
        iPower=0;
    }

    public void runOpMode() throws InterruptedException{

        initialize();



        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(-42, -65, Math.toRadians(270)))
                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                .addTemporalMarker(.5, () -> {
                    switch (b1){
                    case RIGHT:
                        signal=2;
                    case LEFT:
                        signal=4;
                    case CENTER:
                        signal=3;}

                })
                .addSpatialMarker(new Vector2d(-12,-36),()->{
                    signal=5;
                })
                .build();

        TrajectorySequence move2=robot.trajectorySequenceBuilder(new Pose2d(-12, -48,Math.toRadians(90)))
                .setReversed(FALSE)
                .splineTo(new Vector2d(-60,-60),Math.toRadians(225))
                .addSpatialMarker(new Vector2d(-72,-72),()->{
                    signal=1;
                    robot.carousel.setPower(-1);
                })
                .waitSeconds(7)
                .build();

        TrajectorySequence shippingHub=robot.trajectorySequenceBuilder(new Pose2d(14,-65,Math.toRadians(0)))
                .setReversed(TRUE)
                .addTemporalMarker(.1, () -> {
                    iPower=1;
                })
                .splineToLinearHeading(new Pose2d(-50,-60,Math.toRadians(270)),.5)
                .waitSeconds(1)
                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                .addTemporalMarker(.5, () -> {
                    deposit1.dstate1 = depositStateMachine.depositState.HIGH;
                })
                .addSpatialMarker(new Vector2d(-12,-36),()->{
                    signal=5;
                    iPower=0;
                })
                .setReversed(FALSE)
                .splineTo(new Vector2d(-60,-36),3.14/2)
                .addTemporalMarker(10, () -> {
                    deposit1.dstate1 = depositStateMachine.depositState.PRIME;
                })
                .build();

        tState1=trajState.MOVE1;

        while (!opModeIsActive()&&!isStopRequested()) {
            vision1.checkTE();
            b1=vision1.getPosition();
            telemetry.addData("Position",b1);

        }


        waitForStart();


        while (opModeIsActive()){
            switch (tState1){
                case MOVE1:
                    robot.followTrajectorySequenceAsync(move1);
                    if (!robot.isBusy()) {
                        tState1 = trajState.MOVE2;
                    }
                case MOVE2:
                    robot.followTrajectorySequenceAsync(move2);
                    if (!robot.isBusy()){

                        tState1=trajState.SHIPPING;
                    }

                case SHIPPING:
                    robot.carousel.setPower(0);
                    robot.followTrajectorySequenceAsync(shippingHub);
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                    }
                case IDLE:
                    break;
            }

            deposit1.deposit(robot,signal);
            deposit1.updatePID(robot);
            if (deposit1.intakeMode) {
                robot.intake.setPower((iPower)*.75);
            }



            Pose2d poseEstimate = robot.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}