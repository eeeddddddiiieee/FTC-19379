package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.CENTER;
import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.LEFT;
import static org.firstinspires.ftc.teamcode.te.vision.barcodePosition.RIGHT;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.robotControls.depositStateMachine;
import org.firstinspires.ftc.teamcode.te.vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="STATEREDMAIN",group = "drive")
public class StateRedMain extends LinearOpMode {
    //public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    //public depositStateMachine deposit1;
    public int signal;
    public double iPower=-1;
    public vision.barcodePosition b1;
    public boolean readyPark=false;
    public double trackWidth=10.44;
    public enum trajState{
        MOVE1,
        CYCLE1,
        CYCLE2,
        PARK,
        IDLE,
    }
    public trajState tState1;

    public double xPo;
    public double yPo;

    public Pose2d startPose=new Pose2d(12,-65,Math.toRadians(270));
    public Vector2d dumpPose=new Vector2d(4.5,-52);
    public double wait=0;


    public void runOpMode() throws InterruptedException{
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        depositStateMachine deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        deposit1.dstate1= depositStateMachine.depositState.PRIME;
        vision vision1=new vision();
        vision1.initVision(hardwareMap);
        tState1=trajState.MOVE1;
        robot.intakeServo.setPosition(robot.intakeUp);
        robot.setPoseEstimate((new Pose2d(12, -65,Math.toRadians(270))));

        signal=1;
        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(12, -65,Math.toRadians(270)))

                .setReversed(true)
                .addTemporalMarker(.25+wait, () -> {
                    if (b1==LEFT){
                        signal=6;
                    }
                    if (b1==RIGHT){
                        signal=2;
                    }
                    if (b1==CENTER){
                        signal=3;
                    }
                })
                .addTemporalMarker(.75+wait,()->{
                    signal=5;
                })
                .splineTo(dumpPose,Math.toRadians(120))
                .waitSeconds(.25)

                .build();

        TrajectorySequence cycle1=robot.trajectorySequenceBuilder(move1.end())
                //cycle 1
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    signal=1;
                    iPower=-1;
                })
                .splineTo(new Vector2d(14,-65),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(60, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .forward((10),
                        hwMecanum.getVelocityConstraint(30, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )                .splineTo(new Vector2d(44,-65),Math.toRadians(0))
                .setReversed(TRUE)
                .splineTo(new Vector2d(24,-65),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    signal=2;
                })
                .splineTo(dumpPose,Math.toRadians(120),
                        hwMecanum.getVelocityConstraint(40, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    signal=5;
                })
                .waitSeconds(1)

                //cycle 2
                .setReversed(FALSE)
                .addDisplacementMarker(() -> {
                    signal=1;
                    iPower=-1;
                })
                .splineTo(new Vector2d(14,-65),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(60, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .forward((10),
                        hwMecanum.getVelocityConstraint(30, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )                .splineTo(new Vector2d(48,-65),Math.toRadians(0))
                .setReversed(TRUE)
                .splineTo(new Vector2d(24,-65),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    signal=2;
                })
                .splineTo(dumpPose,Math.toRadians(120),
                        hwMecanum.getVelocityConstraint(40, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    signal=5;
                })
                .waitSeconds(1)
                .setReversed(FALSE)

                .build();

        TrajectorySequence cycle2=robot.trajectorySequenceBuilder(cycle1.end())

                //cycle 3
                .splineTo(new Vector2d(14,-65),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(60, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .forward((10),
                        hwMecanum.getVelocityConstraint(30, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .splineTo(new Vector2d(46,-60),Math.toRadians(30))
                .setReversed(TRUE)
                .splineTo(new Vector2d(24,-65),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    signal=2;
                })
                .splineTo(dumpPose,Math.toRadians(120),
                        hwMecanum.getVelocityConstraint(40, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    signal=5;
                })
                .waitSeconds(1)
                .setReversed(FALSE)

                //cycle 4
                .splineTo(new Vector2d(14,-66),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(60, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60))
                .forward((10),
                        hwMecanum.getVelocityConstraint(30, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )                .splineTo(new Vector2d(54,-65),Math.toRadians(0))
                .setReversed(TRUE)
                .splineTo(new Vector2d(24,-66),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    signal=2;
                })
                .splineTo(dumpPose,Math.toRadians(120),
                        hwMecanum.getVelocityConstraint(40, 60, trackWidth),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .waitSeconds(.5)
                .setReversed(false)
                .addDisplacementMarker(()->{
                    readyPark=true;
                })

                .build();

        TrajectorySequence park=robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(()->{
                    signal=5;
                    iPower=0;
                })
                .UNSTABLE_addTemporalMarkerOffset(.5,()->{
                    signal=1;
                    iPower=0;
                })
                .setReversed(false)

                .splineTo(new Vector2d(14,-66),Math.toRadians(0),
                        hwMecanum.getVelocityConstraint(60, 60, 12),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .forward((20),
                        hwMecanum.getVelocityConstraint(65, 60, 12),
                        hwMecanum.getAccelerationConstraint(60)
                )
                .build();



        while (!opModeIsActive()&&!isStopRequested()) {
            vision1.checkTE();
            b1=vision1.getPosition();
            telemetry.addData("position",b1);
            telemetry.addData("ready?","yes");
            telemetry.update();
        }
        waitForStart();


        robot.followTrajectorySequenceAsync(move1);

        while (opModeIsActive()&&!isStopRequested()){

            switch (tState1){
                case MOVE1:{
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                        robot.followTrajectorySequenceAsync(cycle1);
                    }
                }
                case CYCLE1:{
                    if (!robot.isBusy()){
                        tState1=trajState.CYCLE2;
                        robot.followTrajectorySequenceAsync(cycle2);
                    }
                }
                case CYCLE2:{
                    if ((getRuntime()>28||readyPark)&&robot.getPoseEstimate().getX()<14){
                        robot.breakFollowing();
                        tState1=trajState.PARK;
                        robot.followTrajectorySequenceAsync(park);
                    }
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                    }
                }
                case IDLE:{
                    break;
                }
                case PARK:{
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                    }
                }

            }
            if (true&&tState1!=trajState.IDLE){

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
            if (robot.intakeMode==false) {
                robot.intake.setPower(-deposit1.intakePower);
            }
            else if ( robot.intakeMode) {
                robot.intake.setPower(iPower*.55);
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

