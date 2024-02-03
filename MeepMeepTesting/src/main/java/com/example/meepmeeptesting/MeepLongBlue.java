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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35,60,Math.toRadians(90)))
                        //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-31 ,33,Math.toRadians(160)),Math.toRadians(-75))
                        .lineToLinearHeading(new Pose2d(-37.5,33,Math.toRadians(155)))
                        .lineToLinearHeading(new Pose2d(-50,53,Math.toRadians(15)))
                        .lineToLinearHeading(new Pose2d(-45,59,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(12,59,Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(54.5,40.5,Math.toRadians(0)),Math.toRadians(-15))
                        .lineToLinearHeading(new Pose2d(36,30,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(50,12,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(58,14,Math.toRadians(0)))
                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
