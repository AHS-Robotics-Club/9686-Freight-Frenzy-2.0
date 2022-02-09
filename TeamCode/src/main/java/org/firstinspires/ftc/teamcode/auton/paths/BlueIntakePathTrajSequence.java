package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class BlueIntakePathTrajSequence extends SequentialCommandGroup {
    private Pose2d startPose = new Pose2d(9.0, 62.0, 0.0);

    public BlueIntakePathTrajSequence(DropSubsystem drop, Motor intake, SampleMecanumDrive sampleMecanumDrive){

        TrajectorySequence trajectorySequence = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> intake.set(-0.5))
                .lineToLinearHeading(new Pose2d(6.0, 24.0, Math.toRadians(60.0)))   // TODO | Trajectory 1: Go to team hub
                .UNSTABLE_addTemporalMarkerOffset(-1.5, drop::dropTwo)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, drop::dropThree)

                .addTemporalMarker(() -> intake.set(0.5))
                .lineToLinearHeading(new Pose2d(9.0, 62.0, 0.0))           // TODO | Trajectory 2: Go back to Start Position
                .addTemporalMarker(() -> intake.set(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, drop::dropOne)

                .addTemporalMarker(() -> intake.set(-0.45))
                .forward(50)                                                               // TODO | Trajectory 3: Go inside Warehouse
                .waitSeconds(1.0)

                .addTemporalMarker(() -> intake.set(1.0))
                .strafeLeft(30)                                                            // TODO | Trajectory 4: Strafe left to adjust bot
                .back(50)                                                                  // TODO | Trajectory 5: Go back to original position

                .addTemporalMarker(() -> intake.set(-0.5))
                .lineToLinearHeading(new Pose2d(9.0, 49.0, Math.toRadians(0.0)))    // TODO | Trajectory 6: Go to shared hub
                //boolean(if_true then "false")

                .addTemporalMarker(drop::dropTwo)
                .turn(Math.toRadians(100.0))                                               // TODO | Trajectory 7: Turn back towards Shared Hub
                .addTemporalMarker(drop::dropThree)

                .addTemporalMarker(() -> intake.set(0.5))
                .lineToLinearHeading(new Pose2d(9.0, 62.0, 0.0))           // TODO | Trajectory 8: Go back to original position (Copy of Trajectory 2)
                .addTemporalMarker(() -> intake.set(0))
                .addTemporalMarker(drop::dropOne)

                .addTemporalMarker(() -> intake.set(-0.5))
                .forward(50)                                                               // TODO | Trajectory 9: Go park inside warehouse
                .waitSeconds(1)

                .addTemporalMarker(() -> intake.set(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.set(0))
                .build();
    }
}
