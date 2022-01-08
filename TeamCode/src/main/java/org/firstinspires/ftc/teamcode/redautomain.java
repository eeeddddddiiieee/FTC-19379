package org.firstinspires.ftc.teamcode;
//library imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hwMecanum;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
This is the RETARD AUTO. At all expense, please do not use this for the final robot. Time based
park, carousel, idk
//TODO: This auto does not use roadrunner or PID. This is a backup auto.
 */
@Config
@Autonomous(name="redautoprimary",group = "drive")

public class redautomain extends LinearOpMode {
    int width = 320;
    int height = 240;
    TEDetector detector = new TEDetector();
    public void runOpMode() throws InterruptedException, EmptyPathSegmentException {
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        robot.camera.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        robot.camera.setPipeline(detector);
        // Remember to change the camera rotation
        robot.camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);

        FtcDashboard.getInstance().startCameraStream(robot.camera, 20);


        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Analysis", detector.getLocation());
            telemetry.addData("region1", detector.region1value());
            telemetry.addData("region2", detector.region2value());
            telemetry.update();

        }
        waitForStart();

        while (opModeIsActive()&&!isStopRequested()) {
            robot.setPoseEstimate(new Pose2d(-31,-63,Math.toRadians(270)));

            //closes claw and lifts up arm
            //robot.claw.setPosition(robot.servoClosed);
            //robot.claw.setPosition(robot.servoOpen)
            //robot.arm.setPower(-.15);
            //sleep(250);
            //robot.arm.setPower(0);
            robot.claw.setPosition(hwMecanum.CLOSED_CLAW);
            robot.arm2.setPosition(1.02-hwMecanum.inside);
            robot.arm1.setPosition(hwMecanum.inside);

            sleep(300);
            switch (detector.getLocation()) {
                case LEFT:
                    robot.arm2.setPosition(1.02-hwMecanum.low);
                    robot.arm1.setPosition(hwMecanum.low);
                    telemetry.addData("Position:","Left");
                    telemetry.update();
                    break;
                case RIGHT:
                    robot.arm2.setPosition(1.02-hwMecanum.mid);
                    robot.arm1.setPosition(hwMecanum.mid);
                    telemetry.addData("Position:","Center");
                    telemetry.update();
                    break;
                case NONE:
                    robot.arm2.setPosition(1.02-hwMecanum.high);
                    robot.arm1.setPosition(hwMecanum.high);
                    telemetry.addData("Position:","Right");
                    telemetry.update();
                    break;
            }
            robot.camera.stopStreaming();
            sleep(5000);
            //moves forward to the
            Trajectory move1 = robot.trajectoryBuilder(new Pose2d(36, -62,Math.toRadians(270)),true)

                    .splineTo(new Vector2d(-12,-42),Math.toRadians(90))
                    .build();
            robot.followTrajectory(move1);
            robot.claw.setPosition(hwMecanum.OPEN_CLAW);
            sleep(300);

            //robot.claw.setPosition(robot.servoOpen)

            //robot.claw.setPosition(robot.servoOpen);//for red side
            Trajectory move2=robot.trajectoryBuilder(move1.end(),false)
                    .splineTo(new Vector2d(-54,-58),Math.toRadians(270))
                    .build();
            robot.followTrajectory(move2);
            robot.claw.setPosition(.138);
            robot.arm2.setPosition(1.02-hwMecanum.inside);
            robot.arm1.setPosition(hwMecanum.inside);
            robot.carousel.setPower(-.3);
            sleep(6000);
            //robot.carousel.setPower(0);
            //robot.arm.setPower(.35);
            //sleep(350);
            //robot.arm.setPower(0);
            Trajectory move3 = robot.trajectoryBuilder(move2.end(),true)
                    .splineToConstantHeading(new Vector2d(-60,-38),Math.toRadians(0))
                    .build();
            robot.followTrajectory(move3);

            return;
            /*
            robot.setPoseEstimate(new Pose2d(-10,-61,Math.toRadians(90)));

            //closes claw and lifts up arm
            //robot.claw.setPosition(robot.servoClosed);

            //robot.claw.setPosition(robot.servoOpen)
            //robot.arm.setPower(-.15);
            //sleep(250);
            //robot.arm.setPower(0);

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

            return;*/

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
