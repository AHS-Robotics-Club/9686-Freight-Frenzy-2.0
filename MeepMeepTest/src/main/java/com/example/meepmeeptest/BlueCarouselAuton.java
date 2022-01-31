package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCarouselAuton {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-32.0, 62.0, Math.toRadians(270.0)))
//                                        .turn(Math.toRadians(-60.0))
//                                        .back(23.0)
                                        .lineToLinearHeading(new Pose2d(-21.0, 34.0, Math.toRadians(120.0)))
//                                        .forward(23.0)
//                                        .turn(Math.toRadians(60.
                                        .lineToLinearHeading(new Pose2d(-32.0, 57.0, Math.toRadians(180.0)))
                                        .strafeRight(7.0)
//                                        .lineToLinearHeading(new Pose2d(-60.0, 62.0, Math.toRadians(180.0))).waitSeconds(1.5)
                                        .forward(28.0).waitSeconds(1.5)
//                                        .forward(27.0).waitSeconds(1.5)
                                        .strafeLeft(27.0)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}