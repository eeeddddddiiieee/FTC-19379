package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.te.vision;


@Autonomous(name="blueautoprimary",group = "drive")
public class BlueMain extends LinearOpMode {
    public hwMecanum robot;
    public lift robotlift;
    public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public depositStateMachine deposit1;

    public void runOpMode() throws InterruptedException{

        initialize();


        Trajectory move1 = robot.trajectoryBuilder(new Pose2d(38, -62,Math.toRadians(270)),true)

                .splineTo(new Vector2d(14,-40),Math.toRadians(90))
                .build();


        waitForStart();


        while (opModeIsActive()){




        }


    }

    public void initialize(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        vision1=new vision();
        vision1.initVision(hardwareMap);
    }


}

