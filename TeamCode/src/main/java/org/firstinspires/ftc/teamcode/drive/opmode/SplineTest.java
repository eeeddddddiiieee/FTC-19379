package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hwMecanum drive = new hwMecanum(hardwareMap);
        drive.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(6,-65,Math.toRadians(270)));


        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(6,-65,Math.toRadians(270)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(6, -65, Math.toRadians(270)))
                .setReversed(true)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .forward(10)
                .lineTo(new Vector2d(44,-65),
                        SampleMecanumDrive.getVelocityConstraint(25, 60, 12),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .back(24)
                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .forward(10)
                .lineTo(new Vector2d(44,-65),
                        SampleMecanumDrive.getVelocityConstraint(25, 60, 12),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .back(24)
                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                .forward(10)
                .lineTo(new Vector2d(44,-65),
                        SampleMecanumDrive.getVelocityConstraint(25, 60, 12),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .back(24)
                .setReversed(TRUE)
                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))





                .build();

        drive.followTrajectorySequence(traj);

        sleep(2000);


    }
}
