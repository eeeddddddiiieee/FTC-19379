package com.example.meepmeep;


import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.*;
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
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12    )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31, -63, Math.toRadians(270)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-12,-44),Math.toRadians(90))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-54,-54),Math.toRadians(270))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-60,-36),Math.toRadians(0))

                                .build()
                )
                .start();
    }
}