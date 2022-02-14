package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.CENTER;
import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.LEFT;
import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.RIGHT;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.te.vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="testauto",group = "drive")
public class testAuto extends LinearOpMode {
    //public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    //public depositStateMachine deposit1;
    public int signal;
    public double iPower;
    public vision.barcodePosition b1;
    public enum trajState{
        MOVE1,
        FORWARDBACK,
        SHIPPING,
        IDLE
    }
    public trajState tState1;

    public double xPo;
    public double yPo;

    public Pose2d startPose=new Pose2d(-6,-65,Math.toRadians(270));

    public void runOpMode() throws InterruptedException{
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        depositStateMachine deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        deposit1.dstate1= depositStateMachine.depositState.PRIME;
        vision vision1=new vision();
        vision1.initVision(hardwareMap);
        tState1=trajState.MOVE1;

        signal=1;


        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(6, -65,Math.toRadians(270)))

                .setReversed(true)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .addTemporalMarker(.5, () -> {
                    switch (b1){

                        case RIGHT:
                            signal=2;
                        case LEFT:
                            signal=4;
                        case CENTER:
                            signal=4;
                    }

                })
                .addTemporalMarker(2,()->{
                    signal=5;
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence fwBw=robot.trajectorySequenceBuilder(new Pose2d(-12, -48,Math.toRadians(270)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))

                .forward(10)
                .addTemporalMarker(.5, () -> {
                    iPower=1;
                })
                .addTemporalMarker(.5, () -> {
                    signal=1;
                })
                .lineTo(new Vector2d(44,-65),
                        hwMecanum.getVelocityConstraint(25, 60, 12),
                        hwMecanum.getAccelerationConstraint(20)
                )

                .build();

        robot.setPoseEstimate(new Pose2d(6,-65,Math.toRadians(270)));
        TrajectorySequence shippingHub=robot.trajectorySequenceBuilder(new Pose2d(14,-65,Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(24,-65),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(40,60,12),
                        hwMecanum.getAccelerationConstraint(20)
                )
                .setReversed(false)
                .back(10)

                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .addTemporalMarker(5, () -> {
                    deposit1.dstate1 = depositStateMachine.depositState.HIGH;

                })
                .addSpatialMarker(new Vector2d(-12,-36),()->{
                    signal=1;
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .build();




        while (!opModeIsActive()&&!isStopRequested()) {
            vision1.checkTE();
            b1=vision1.getPosition();
            telemetry.addData("position",b1);
            telemetry.addData("ready?","yes");
            telemetry.update();

        }
        waitForStart();




        robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(new Pose2d(6, -65,Math.toRadians(270)))

                .setReversed(true)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .addTemporalMarker(.5, () -> {
                    if (b1==LEFT){
                        signal=4;
                    }
                    if (b1==RIGHT){
                        signal=2;
                    }
                    if (b1==CENTER){
                        signal=3;
                    }

                })
                .addTemporalMarker(2,()->{
                    signal=5;
                })
                .waitSeconds(1)
                .build());
        while (opModeIsActive()&&!isStopRequested()){

            switch (tState1){
                case MOVE1:{


                    if (!robot.isBusy()){
                        tState1=trajState.FORWARDBACK;
                        robot.followTrajectorySequenceAsync(fwBw);


                    }
                }
                case FORWARDBACK:{
                    if (!robot.isBusy()){
                        tState1=trajState.SHIPPING;
                        robot.followTrajectorySequenceAsync(shippingHub);
                    }

                }
                case SHIPPING:{
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                    }


                }
                case IDLE:{
                    break;
                }

            }

            robot.update();
            telemetry.update();
            deposit1.deposit(robot,signal);
            deposit1.updatePID(robot);

            if (robot.bucketSensor.getDistance(DistanceUnit.MM)<50){
                robot.isCargo=TRUE;
            }
            else {
                robot.isCargo=FALSE;
            }



            Pose2d poseEstimate = robot.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("liftstate",deposit1.robotlift.getHeight());
            telemetry.update();
        }


    }




}

