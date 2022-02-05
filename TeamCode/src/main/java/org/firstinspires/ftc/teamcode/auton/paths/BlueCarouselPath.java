package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.time.Instant;

@Config
public class BlueCarouselPath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(-32.0, 62.0, Math.toRadians(270.0));
    private long LIFT_UP;
    private long LIFT_DOWN;

    public BlueCarouselPath(Constants.HubLevel level, MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake, DuckySpinnerSubsystem duckySpinner, LiftSubsystemNoPID lift) {

        switch(level) {
            case MID:
                LIFT_UP = Constants.LiftConstants.MID_GOAL_UP;
                LIFT_DOWN = Constants.LiftConstants.MID_GOAL_DOWN;
            case HIGH:
                LIFT_UP = Constants.LiftConstants.HIGH_GOAL_UP;
                LIFT_DOWN = Constants.LiftConstants.HIGH_GOAL_DOWN;
            default:
                LIFT_UP = 0;
                LIFT_DOWN = 0;
        }

        drive.setPoseEstimate(startPose);
        Trajectory traj0 = drive.trajectoryBuilder(startPose) // Go to allianace hub
                .lineToLinearHeading(new Pose2d(-21.0, 34.0, Math.toRadians(120.0)))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end()) // Go to ducky spinner
                .lineToLinearHeading(new Pose2d(-62.0, 58.0, Math.toRadians(140.0)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // Go to parking location
                .lineToLinearHeading(new Pose2d(-60.0, 35.5, Math.toRadians(0))) // Park
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()) // Move forward to pick up freight
                .forward(37)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()) // Return to parking location
                .lineToLinearHeading(new Pose2d(-60.0, 35.5, Math.toRadians(0))) // Park
                .build();

        addCommands(
              new InstantCommand(() -> intake.set(-0.5)),       // Set intake on to pick up any items
                //region
//              new TrajectoryFollowerCommand(drive, traj0)       // Go to Alliance Hub
//                .alongWith(new InstantCommand(drop::dropTwo)    // Set carriage parallel to ground so it doesn't fall out
//                .andThen(new WaitCommand(1200)          // Wait so that block doesn't get launched
//                .andThen(new InstantCommand(drop::dropThree)))), // Bring down carriage
                //endregion
              new TrajectoryFollowerCommand(drive, traj0)       // Go to Alliance Hub
                .alongWith(new InstantCommand(drop::dropTwo),    // Set carriage parallel to ground so it doesn't fall out
              new InstantCommand(lift::motorUp).andThen(new WaitCommand(LIFT_UP)).andThen(new InstantCommand(lift::motorStop))), // Lift up lift to right level
              new InstantCommand(drop::dropThree), // Drop block

              new InstantCommand(lift::motorDown).andThen(new WaitCommand(LIFT_DOWN)).andThen(new InstantCommand(lift::motorStop)), // Bring down lift to init pos

              new ParallelCommandGroup(
                  new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)), // Set intake to put back carriage
                  new InstantCommand(drop::dropOne) // Reset carriage
              ),
              new TrajectoryFollowerCommand(drive, traj1)                   // Go to the carousel
                .andThen(new RunCommand(() -> duckySpinner.run(0.4)) // Run ducky spinner motor after going there
                        .raceWith(new WaitCommand(2950))),          // Wait till the duck drops

              new TrajectoryFollowerCommand(drive, traj2),                  // Go to parking (Intermediate to shared hub)

              new InstantCommand(() -> intake.set(-0.5)),                   // Set intake to pick up shipping element
              new TrajectoryFollowerCommand(drive, traj3)                   // Go towards the shipping hub
                      .andThen(new InstantCommand(drop::dropTwo)),          // Set carriage parallel to ground so shipping element doesn't drop out

              //region
              // new TurnCommand(drive, Math.toRadians(130.0))                 // Turn back to shared hub
              //   .andThen(new WaitCommand(1200)                      // Wait so element doesn't launch
              //   .andThen(new InstantCommand(drop::dropThree))),             // Drop element
              //endregion

              new TurnCommand(drive, Math.toRadians(130.0))                 // Turn back to shared hub
                .andThen(new WaitCommand(1200),                      // Wait so element doesn't launch
              new InstantCommand(lift::motorUp).andThen(new WaitCommand(LIFT_UP)).andThen(new InstantCommand(lift::motorStop))), // Lift up lift to right level
              new InstantCommand(drop::dropThree), // Drop block
              new InstantCommand(lift::motorDown).andThen(new WaitCommand(LIFT_DOWN)).andThen(new InstantCommand(lift::motorStop)), // Bring down lift to init pos

              new TrajectoryFollowerCommand(drive, traj4),                   // Go to parking
              new InstantCommand(() -> intake.set(0)), // Stop intake
              new InstantCommand(drop::dropOne) // Reset carriage
        );
    }
}
