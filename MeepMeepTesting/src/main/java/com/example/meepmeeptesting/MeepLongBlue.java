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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36,60,Math.toRadians(-90)))
                        //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
                        .lineToLinearHeading(new Pose2d(-50,51.5,Math.toRadians(90)))
                        //
                        .lineToLinearHeading(new Pose2d(-39,40,Math.toRadians(90)))
                        //.lineToLinearHeading(new Pose2d(-35,43 ,Math.toRadians(140)))
                        .lineToLinearHeading(new Pose2d(-40,55,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60,55,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60,16,Math.toRadians(90)))

                        .lineToLinearHeading(new Pose2d(-49,11,Math.toRadians(0)))

                        .lineToLinearHeading(new Pose2d(-35,11,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(12,11,Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(54.5,35,Math.toRadians(0)),Math.toRadians(10))
                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
