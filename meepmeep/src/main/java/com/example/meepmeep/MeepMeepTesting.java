package com.example.meepmeep;


import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String args[]){
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity redMain = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(12,14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(6, -65, Math.toRadians(270)))
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                                .setReversed(FALSE)
                                .splineToSplineHeading(new Pose2d(14,-65,Math.toRadians(0)),Math.toRadians(-20))
                                .forward(10)
                                .lineTo(new Vector2d(44,-65),
                                        SampleMecanumDrive.getVelocityConstraint(25, 60, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(20)
                                )
                                .back(24)
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))





                                .build()
                );

        RoadRunnerBotEntity blueMain = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(12,14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(6, 65, Math.toRadians(-270)))
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,40),Math.toRadians(-90))
                                .setReversed(FALSE)
                                .splineToSplineHeading(new Pose2d(14,65,Math.toRadians(-0)),Math.toRadians(20))
                                .forward(10)
                                .lineTo(new Vector2d(44,65),
                                        SampleMecanumDrive.getVelocityConstraint(25, 60, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(20)
                                )
                                .back(24)
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,40),Math.toRadians(-90))

                                .build()
                );

        RoadRunnerBotEntity red2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(12,14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -65, Math.toRadians(270)))
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                                .setReversed(FALSE)
                                .splineTo(new Vector2d(-60,-60),Math.toRadians(225))
                                .waitSeconds(1)
                                .setReversed(TRUE)
                                .splineToLinearHeading(new Pose2d(-50,-60,Math.toRadians(270)),.5)
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                                .setReversed(FALSE)
                                .splineTo(new Vector2d(-60,-36),3.14/2)


                                .build()
                );

        RoadRunnerBotEntity blue2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(12,14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, 65, Math.toRadians(-270)))
                                .setReversed(TRUE)
                                .splineTo(new Vector2d(-12,40),Math.toRadians(-90))
                                .setReversed(FALSE)
                                .splineTo(new Vector2d(-60,60),Math.toRadians(-225))
                                .waitSeconds(1)
                                .setReversed(TRUE)
                                .splineToLinearHeading(new Pose2d(-50,60,Math.toRadians(-270)),-.5)
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-12,40),Math.toRadians(-90))
                                .setReversed(FALSE)
                                .splineTo(new Vector2d(-60,36),-3.14/2)


                                .build()
                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blue2)
                .addEntity(redMain)
                .addEntity(blueMain)
                .addEntity(red2)

                .start();

    }
}