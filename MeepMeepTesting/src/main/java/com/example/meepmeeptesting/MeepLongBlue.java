package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import jdk.tools.jlink.internal.plugins.VendorVersionPlugin;

public class MeepLongBlue{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36,61,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-46, 39, Math.toRadians(90)))
                        .waitSeconds(0.1)
                        .lineToLinearHeading(new Pose2d(-46, 45,Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(-23,61, Math.toRadians(0)), Math.toRadians(0))
                        //.splineToSplineHeading(new Pose2d(25,40,Math.toRadians(0)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(47,28, Math.toRadians(0)),Math.toRadians(0))
                        .waitSeconds(3)
                        .lineToLinearHeading(new Pose2d(-32,37,Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-50,35.5, Math.toRadians(0)), Math.toRadians(10))
                        .lineToLinearHeading(new Pose2d(-60, 35.5, Math.toRadians(0)))
                        .waitSeconds(3)
                        .lineToLinearHeading(new Pose2d(-50,35.5, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-23,60, Math.toRadians(0)), Math.toRadians(15))
                        .lineToLinearHeading(new Pose2d(0,60,Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(47,28, Math.toRadians(0)),Math.toRadians(-15))
                        .lineToLinearHeading(new Pose2d(40,12,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(60, 12,Math.toRadians(0)))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
