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
@Autonomous(name="REDSECONDARY",group = "drive")
public class red2 extends LinearOpMode {
    //public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    //public depositStateMachine deposit1;
    public int signal;
    public double iPower;
    public vision.barcodePosition b1;
    public enum trajState{
        MOVE1,
        DUCKS,
        CYCLE2,
        IDLE
    }
    public trajState tState1;

    public double xPo;
    public double yPo;

    public Pose2d startPose=new Pose2d(-36,-65,Math.toRadians(270));

    public void runOpMode() throws InterruptedException{
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        depositStateMachine deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        deposit1.dstate1= depositStateMachine.depositState.PRIME;
        vision vision1=new vision();
        vision1.initVision(hardwareMap);
        tState1=trajState.MOVE1;
        robot.setPoseEstimate((new Pose2d(-36, -65,Math.toRadians(270))));

        signal=1;


        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(-36, -65,Math.toRadians(270)))

                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-46),Math.toRadians(90),hwMecanum.getVelocityConstraint(35, 60, 12),
                        hwMecanum.getAccelerationConstraint(40)
                )
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
                .addTemporalMarker(2.5,()->{
                    signal=5;
                })
                .waitSeconds(.25)
                .build();

        TrajectorySequence ducks=robot.trajectorySequenceBuilder(move1.end())
                .setReversed(FALSE)
                .splineTo(new Vector2d(-60,-61),Math.toRadians(225),hwMecanum.getVelocityConstraint(45, 60, 12),
                        hwMecanum.getAccelerationConstraint(50)
                )
                .waitSeconds(1)
                .addTemporalMarker(4, () -> {
                    robot.carousel.setPower(.6);
                })
                .addTemporalMarker(10, () -> {
                    robot.carousel.setPower(0);

                })
                .waitSeconds(10)
                .setReversed(false)
                .build();

        TrajectorySequence cycle2=robot.trajectorySequenceBuilder((ducks.end()))
                .setReversed(TRUE)
                .splineTo(new Vector2d(-59,-35.5),3.14/2)
                .addTemporalMarker(4, () -> {
                    deposit1.setState(depositStateMachine.depositState.PRIME);
                    robot.intake.setPower(0);
                })
                .build();






        while (!opModeIsActive()&&!isStopRequested()) {
            vision1.checkTE();
            b1=vision1.getPosition();
            telemetry.addData("position",b1);
            telemetry.addData("ready?","yes");
            telemetry.update();

        }
        waitForStart();

        robot.setPoseEstimate((new Pose2d(-36, -65,Math.toRadians(270))));



        robot.followTrajectorySequenceAsync(move1);
        while (opModeIsActive()&&!isStopRequested()){

            switch (tState1){
                case MOVE1:{


                    if (!robot.isBusy()){
                        tState1=trajState.DUCKS;
                        robot.followTrajectorySequenceAsync(ducks);


                    }
                }
                case DUCKS:{
                    if (!robot.isBusy()){
                        tState1=trajState.CYCLE2;
                        robot.followTrajectorySequenceAsync(cycle2);
                    }

                }
                case CYCLE2:{
                    if (!robot.isBusy()){
                        tState1=trajState.IDLE;
                    }
                }
                case IDLE:{
                    break;
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

