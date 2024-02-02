package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import jdk.tools.jlink.internal.plugins.VendorVersionPlugin;

public class BlueShorttest{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11,61,Math.toRadians(90)))
                                .setReversed(true)
                        .splineToLinearHeading(new Pose2d(12,28,Math.toRadians(-155)),Math.toRadians(-110))
                        .splineToLinearHeading(new Pose2d(12,28,Math.toRadians(-25)),Math.toRadians(-75))
                        .lineToLinearHeading(new Pose2d(20,55,Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(30,55,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36,50,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52,59,Math.toRadians(0)))

                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}








