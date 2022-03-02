package org.firstinspires.ftc.teamcode.regionalsArchive;

import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.LEFT;
import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.RIGHT;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.te.vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*2@Config
@Autonomous(name="Redautoprimary",group = "drive")
public class RedMain extends LinearOpMode {
    //public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    //public depositStateMachine deposit1;
    public boolean signal;
    public double iPower;
    public vision.barcodePosition b1;

    public enum trajState{
        MOVE1,
        FORWARDBACK,
        SHIPPING,
        IDLE
    }
    public double xPo;
    public double yPo;

    public trajState tState1;
    public Pose2d startPose=new Pose2d(-6,-65,Math.toRadians(270));

    public void runOpMode() throws InterruptedException{
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        depositStateMachine deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        deposit1.dstate1= depositStateMachine.depositState.PRIME;
        vision vision1=new vision();
        vision1.initVision(hardwareMap);

        xPo=44;
        yPo=-65;
        iPower=0;




        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(6, -65,Math.toRadians(270)))

                .setReversed(true)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .addTemporalMarker(.5, () -> {
                    switch (b1){

                        case RIGHT:
                            deposit1.dstate1 = depositStateMachine.depositState.HIGH;
                        case LEFT:
                            deposit1.dstate1 = depositStateMachine.depositState.PRIME;
                        case CENTER:
                            deposit1.dstate1 = depositStateMachine.depositState.MID;
                    }

                })
                .addSpatialMarker(new Vector2d(-12,-36),()->{
                    signal=true;
                })
                .build();

        TrajectorySequence fwBw=robot.trajectorySequenceBuilder(new Pose2d(-12, -48,Math.toRadians(270)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))

                .forward(10)
                .addTemporalMarker(.5, () -> {
                    iPower=1;
                })
                .addTemporalMarker(.5, () -> {
                    signal=false;
                })
                .lineTo(new Vector2d(xPo,yPo),
                        hwMecanum.getVelocityConstraint(25, 60, 12),
                        hwMecanum.getAccelerationConstraint(20)
                )

                .build();

        TrajectorySequence shippingHub=robot.trajectorySequenceBuilder(new Pose2d(14,-65,Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(24,-65),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(40,60,12),
                        hwMecanum.getAccelerationConstraint(20)
                )
                .back(10)

                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .addTemporalMarker(5, () -> {
                    deposit1.dstate1 = depositStateMachine.depositState.HIGH;

                })
                .addSpatialMarker(new Vector2d(-12,-36),()->{
                    signal=true;
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .build();

        robot.setPoseEstimate(new Pose2d(6,-65,Math.toRadians(270)));
        tState1=trajState.MOVE1;

        while (!opModeIsActive()&&!isStopRequested()) {
           vision1.checkTE();
        b1=vision1.getPosition();
        telemetry.addData("position",b1);
        telemetry.addData("ready?","yes");
        telemetry.update();

        }


        waitForStart();




        while (opModeIsActive()&&!isStopRequested()){
            switch (tState1){
                case MOVE1:
                    robot.followTrajectorySequenceAsync(move1);
                    telemetry.addData("state","1");
                    if (!robot.isBusy()) {
                        tState1 = trajState.FORWARDBACK;
                    }
                case FORWARDBACK:
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(new Pose2d(-12, -48,Math.toRadians(90)))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))

                            .forward(10)
                            .addTemporalMarker(.5, () -> {
                                iPower=1;
                            })
                            .addTemporalMarker(.5, () -> {
                                signal=false;
                            })
                            .lineTo(new Vector2d(xPo,yPo),
                                    hwMecanum.getVelocityConstraint(25, 60, 12),
                                    hwMecanum.getAccelerationConstraint(20)
                            )

                            .build());
                    if (!robot.isBusy()&&xPo<51){
                        xPo+=2;
                        yPo+=2;
                        tState1=trajState.SHIPPING;
                    }
                    else {
                        robot.breakFollowing();
                        tState1=trajState.IDLE;
                    }
                case SHIPPING:
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(new Pose2d(14,-65,Math.toRadians(0)))
                            .setReversed(true)
                            .splineTo(new Vector2d(24,-65),Math.toRadians(0),
                                    hwMecanum.getVelocityConstraint(40,60,12),
                                    hwMecanum.getAccelerationConstraint(20)
                            )
                            .back(10)

                            .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                            .addTemporalMarker(5, () -> {
                                deposit1.dstate1 = depositStateMachine.depositState.HIGH;

                            })
                            .addSpatialMarker(new Vector2d(-12,-36),()->{
                                signal=true;
                            })
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                            .build());
                    if (!robot.isBusy()){
                        tState1=trajState.FORWARDBACK;
                    }
                case IDLE:
                    break;
            }

            deposit1.deposit(robot,signal);
            deposit1.updatePID(robot);
            robot.update();

            if (deposit1.intakeMode) {
                robot.intake.setPower((iPower)*.75);
            }
            else {
                robot.intake.setPower(deposit1.intakePower);
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
*/
