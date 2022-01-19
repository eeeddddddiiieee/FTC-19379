package org.firstinspires.ftc.teamcode;
//library imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hwMecanum;
//import deez nuts



/*
This is the test AUTO. At all expense, please do not use this for the final robot. Time based
park, carousel, idk
TODO: This auto does not use roadrunner or PID. This is a backup auto.
 */
public class auto1 extends LinearOpMode {
    hwMecanum robot;
    lift robotlift;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public void runOpMode() throws InterruptedException{
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        robotlift=new lift(hardwareMap);
        robotlift.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            robotlift.setPosition(lift.liftHeight.High);
        }


    }


}
