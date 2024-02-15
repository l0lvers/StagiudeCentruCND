package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepShortRedCenter
//-9 -70
{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11,-61,Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(48, -35, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(25, -24))
                        .waitSeconds(1)
                        //stack 1
                        .lineToConstantHeading(new Vector2d(25, -59))
                        .lineToConstantHeading(new Vector2d(-14, -59))
                        .lineToLinearHeading(new Pose2d(-16, -59, Math.toRadians(-23)))
                        .waitSeconds(1)
                        //score 2
                        .lineToLinearHeading(new Pose2d(-14, -59, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(30, -59))
                        .lineToConstantHeading(new Vector2d(48, -35))
                        //stack 2
                        .lineToConstantHeading(new Vector2d(30, -59))
                        .lineToConstantHeading(new Vector2d(-14, -59))
                        .lineToLinearHeading(new Pose2d(-16, -59, Math.toRadians(-23)))
                        .waitSeconds(1)
                        //score 2
                        .lineToLinearHeading(new Pose2d(-14, -59, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(30, -59))
                        .lineToConstantHeading(new Vector2d(48, -35))
                        .lineToLinearHeading(new Pose2d(41, -35, Math.toRadians(0)))
                        //park
                        .lineToConstantHeading(new Vector2d(41, -59))
                        .lineToConstantHeading(new Vector2d(57, -59))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


