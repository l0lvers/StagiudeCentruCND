
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ShortRedR {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11,-61,Math.toRadians(90)))
                        //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
                        .lineToLinearHeading(new Pose2d(11,-44,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(22,-44,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(22,-34,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(22,-44,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(51,-44,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(44,-44,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(44,-60,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(57,-60,Math.toRadians(0)))


                        .build()
                    );


            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
                        //ceau lorena




    }

