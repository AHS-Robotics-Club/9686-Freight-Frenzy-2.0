package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedIntakeAutonNoTune {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.0, -62.0, 0.0))
//                                .strafeLeft(5.0)
                                .lineToLinearHeading(new Pose2d(9.0, -57.0, Math.toRadians(-60.0)))
//                                .turn(Math.toRadians(-60.0))
                                .back(23.0)
                                // Bring carriage down
                                .forward(23.0)
//                                .turn(Math.toRadians(60.0))
//                                .strafeRight(5.0)
                                .lineToLinearHeading(new Pose2d(9.0, -62.0, 0.0))
                                .forward(36.0).waitSeconds(0.2)
                                // Intake On
                                .forward(12.0)
                                .back(12.0)
                                // Intake off, lift up carriage
                                .back(36.0)
//                                .strafeLeft(5.0)
//                                .turn(Math.toRadians(-60.0))
//                                .lineToLinearHeading(new Pose2d(9.0, -57.0, Math.toRadians(-60.0)))
//                                .back(23.0)
                                .build()
                );

        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12.0, 62.0, Math.toRadians(90.0)))
                                .back(27.0)
                                .turn(Math.toRadians(90.0))
                                .forward(50.0)
                                .build()
                );

        RoadRunnerBotEntity thirdBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0.0, 62.0, Math.toRadians(0.0)))
                            .forward(36.0)
                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(secondBot)
                .addEntity(thirdBot)
                .start();
    }
}