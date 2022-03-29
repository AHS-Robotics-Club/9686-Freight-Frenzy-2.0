package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCarouselAutonVision {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73.17, 73.17, Math.toRadians(360), Math.toRadians(360), 9)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-40.0, 62.0, Math.toRadians(270.0)))
//                                .strafeLeft(5.0)
                                        .lineToLinearHeading(new Pose2d(-24.0, 34.0, Math.toRadians(135.0))) // Drop init freight
//                                .splineTo(new Vector2d(-6.0, 34.0), Math.toRadians(60.0))
                                        .lineToLinearHeading(new Pose2d(-40.0, 62.0, Math.toRadians(180.0))) // Go back to start pos
                                        .forward(20)
//                                .splineToLinearHeading(new Pose2d(49.0, 62.0, 0.0)).waitSeconds(0.2) // Go into warehouse
//                                .lineToLinearHeading(new Pose2d(59.0, 62.0, 0.0)) // Get blocks
//                                .splineToSplineHeading(new Pose2d(-6.0, 34.0, Math.toRadians(60.0)), 360.0)
//                                .lineToLinearHeading(new Pose2d(9.0, 62.0, 0.0)) // Go to start pos
                                        .strafeLeft(26.0)// Drop new freight
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
