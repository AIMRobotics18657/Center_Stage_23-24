package com.aimrobotics.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.25, 15)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -64.5, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(30)), Math.toRadians(0))
//                                        .lineTo(new Vector2d(-36, -38))
//                                        .splineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(150)), Math.toRadians(0))
                                        .lineToSplineHeading(new Pose2d(-40, -59, Math.toRadians(0)))
                                        .lineTo(new Vector2d(12, -59))
                                        .splineToConstantHeading(new Vector2d(30, -36), Math.toRadians(0))
                                        .lineTo(new Vector2d(50, -36))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
