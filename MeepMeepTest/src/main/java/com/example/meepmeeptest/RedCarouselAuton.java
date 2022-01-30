package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedCarouselAuton {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .setDimensions(13, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-28.0, -62.0, Math.toRadians(90.0)))
                                .lineToLinearHeading(new Pose2d(-21.0, -34.0, Math.toRadians(240.0)))

                                /* Adjust Using Wall */
                                // .lineToLinearHeading(new Pose2d(-32.0, 57.0, Math.toRadians(180.0)))
                                // .strafeRight(7.0)
                                // .forward(28.0).waitSeconds(1.5)
                                // .strafeLeft(27.0)
                                // .build()

                                // Go directly to ducky spinner
                                .lineToLinearHeading(new Pose2d(-62.0, -58.0, Math.toRadians(270.0)))

                                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(0))) // Park
                                .forward(37)
                                .turn(Math.toRadians(230.0))


                                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(0))) // Park
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}