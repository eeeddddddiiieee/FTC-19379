package com.example.meepmeep;


import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class MeepMeepTesting {
    public static void main(String args[]){
        MeepMeep mm = new MeepMeep(600)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(90)))
                                .forward(16)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(55, -58), Math.toRadians(0))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(60,-37),Math.toRadians(0))
                                .build()
                )
                .start();
    }
}