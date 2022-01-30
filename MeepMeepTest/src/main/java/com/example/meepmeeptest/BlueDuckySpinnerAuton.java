package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueDuckySpinnerAuton {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73.17, 73.17, Math.toRadians(360), Math.toRadians(360), 9)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-12.0, 62.0, Math.toRadians(90)))
                                        .lineTo(new Vector2d(-12.0, 37.0)) // Put init block
                                        .lineToSplineHeading(new Pose2d(-60.0, 56.0, Math.toRadians(190))) // Ducky Spinner on
                                        .lineToLinearHeading(new Pose2d(-58.0, 34.0, Math.toRadians(0))) // Park
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
