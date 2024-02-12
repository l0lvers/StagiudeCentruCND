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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36,-61,Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-35,31,Math.toRadians(90)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-36,50,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-50,50,Math.toRadians(90)))
                        .lineTo(new Vector2d(42, 42))
                        .lineTo(new Vector2d(34,34))
                        .lineTo(new Vector2d(-50,34))
                        .lineTo(new Vector2d(-50,35))
                        .lineTo(new Vector2d(-61, 35))
                        .waitSeconds(3)
                        .lineTo(new Vector2d(45,35))
                        .waitSeconds(3)
                        .lineTo(new Vector2d(42, 35))
                        .lineTo(new Vector2d(42,13))
                        .lineTo(new Vector2d(61,13))
                        .waitSeconds(3)
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
