package org.firstinspires.ftc.teamcode;
//library imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hwMecanum;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/*
This is the RETARD AUTO. At all expense, please do not use this for the final robot. Time based
park, carousel, idk
//TODO: This auto does not use roadrunner or PID. This is a backup auto.
 */
@Config
@Autonomous(name="redmainauto",group = "drive")

public class autoredmain extends LinearOpMode {
    public void runOpMode() throws InterruptedException, EmptyPathSegmentException {
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.setPoseEstimate(new Pose2d(-10,-61,Math.toRadians(90)));

            //closes claw and lifts up arm
            robot.claw.setPosition(robot.servoClosed);

            //robot.claw.setPosition(robot.servoOpen)
            robot.arm.setPower(-.15);
            sleep(250);
            robot.arm.setPower(0);

            //moves forward to the
            Trajectory move1 = robot.trajectoryBuilder(new Pose2d(-10, -61,Math.toRadians(90)))
                    .forward(20)
                    .build();
            robot.followTrajectory(move1);
            //robot.claw.setPosition(robot.servoOpen)


            robot.claw.setPosition(robot.servoOpen);
            sleep(100);

            //for red side
            //TODO: comment this out if we are on blue and vice versa
            Trajectory move2=robot.trajectoryBuilder(move1.end(),true)
                    .splineTo(new Vector2d(-55, -58), Math.toRadians(-90))
                    .build();
            robot.followTrajectory(move2);
            robot.carousel.setPower(.4);
            sleep(5000);
            robot.carousel.setPower(0);
            sleep(1000);
            robot.arm.setPower(.35);
            sleep(350);
            robot.arm.setPower(0);
            Trajectory move3 = robot.trajectoryBuilder(move2.end())
                    .splineTo(new Vector2d(-55,-37),Math.toRadians(90))
                    .build();
            robot.followTrajectory(move3);

            return;

            //for blue side
            /*
            Trajectory move2Blue=robot.trajectoryBuilder(new Pose2d(0,0,0),true)
                    .splineToSplineHeading(new Pose2d(4, 0, Math.toRadians(45)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(6, -48, Math.toRadians(90)), Math.toRadians(0))
                    .build();
            robot.followTrajectory(move2);
            robot.carousel.setPower(.5);
            sleep(3);
            robot.carousel.setPower(0);
            Trajectory move3Blue=robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .splineTo(new Vector2d(24,-52),Math.toRadians(0))
                    .build();
            robot.followTrajectory(move3);

             */

            /*
            Trajectory line1 = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .lineTo(new Vector2d(0, 0))
                    .build();

            Trajectory left1 = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .strafeLeft(50)
                    .build();

            Trajectory strafe1 = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .strafeTo(new Vector2d(0, 50))
                    .build();

            Trajectory spline1 = robot.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .splineTo(new Vector2d(50, 50), Math.toRadians(90))
                    .build();*/
        }









    }
}
