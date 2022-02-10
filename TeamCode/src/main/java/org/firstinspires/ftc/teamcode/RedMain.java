package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.TRUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.te.vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Redautoprimary",group = "drive")
public class RedMain extends LinearOpMode {
    public hwMecanum robot;
    public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public depositStateMachine deposit1;

    enum trajState{
        MOVE1,
        FORWARDBACK,
        SHIPPING,
        IDLE
    }
    public Pose2d startPose=new Pose2d(-6,65,Math.toRadians(270));

    public void runOpMode() throws InterruptedException{

        initialize();

        deposit1=new depositStateMachine();


        TrajectorySequence move1 = robot.trajectorySequenceBuilder(new Pose2d(38, -62,Math.toRadians(270)))

                .setReversed(true)
                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .build();

        TrajectorySequence fwBw=robot.trajectorySequenceBuilder(move1.end())
                .forward(10)
                .lineTo(new Vector2d(44,-65),
                        hwMecanum.getVelocityConstraint(25, 60, 12),
                        hwMecanum.getAccelerationConstraint(20)
                )
                .back(24)
                .build();

        TrajectorySequence shippingHub=robot.trajectorySequenceBuilder((fwBw.end()))
                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .build();



        waitForStart();


        while (opModeIsActive()){

            deposit1.deposit(robot);
            deposit1.updatePID(robot);



        }


    }

    public void initialize(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        deposit1=new depositStateMachine();
        deposit1.initDeposit();
        vision1=new vision();
        vision1.initVision(hardwareMap);
    }


}

