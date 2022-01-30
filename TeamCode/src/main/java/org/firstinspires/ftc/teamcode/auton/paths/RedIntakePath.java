package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auton.e_vision.FFCapstoneDetector;
import org.firstinspires.ftc.teamcode.auton.e_vision.FFCapstonePipeline;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.HashMap;

@Config
public class RedIntakePath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(9.0, -62.0, Math.toRadians(90));

    public RedIntakePath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-6.0, 34.0, Math.toRadians(60.0))) // Drop init freight
//                .lineToLinearHeading(new Pose2d(0.0, 30.0, Math.toRadians(60.0))) // Drop init freight
//                .lineToLinearHeading(new Pose2d(6.0, 30.0, Math.toRadians(60.0))) // Drop init freight
                .lineToLinearHeading(new Pose2d(6.0, -32.0, Math.toRadians(-60.0))) // Drop init freight
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(9.0, -62.0, 0.0)) // Go back to start pos
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // Go into warehouse
                .forward(50)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(30)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToLinearHeading(new Pose2d(9.0, 62.0, 0.0)) // Exit warehouse
//                .lineToLinearHeading(new Pose2d(9.0, 80.0, 0.0)) // Exit warehouse
                .back(50)
                .build();

        Trajectory traj0_1 = drive.trajectoryBuilder(traj4.end())
//                .lineToLinearHeading(new Pose2d(9.0, 58.0, Math.toRadians(0.0))) // Drop init freight after exiting warehouse
                .lineToLinearHeading(new Pose2d(9.0, -49.0, Math.toRadians(0.0))) // Drop init freight after exiting warehouse
                .build();

        Trajectory traj0_2 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-6.0, -34.0, Math.toRadians(-60.0))) // Drop init freight after exiting warehouse
                .build();

        addCommands(
                new InstantCommand(() -> intake.set(-0.5)),
                new TrajectoryFollowerCommand(drive, traj0).alongWith(new InstantCommand(drop::dropTwo).andThen(new WaitCommand(1200).andThen(new InstantCommand(drop::dropThree)))),
                new InstantCommand(() -> intake.set(0.5)),
                new TrajectoryFollowerCommand(drive, traj1)
                    .alongWith(new InstantCommand(drop::dropFour)),
                new ParallelCommandGroup(
                    new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)),
                    new InstantCommand(drop::dropOne)
                ),
                new InstantCommand(() -> intake.set(-0.45)),
                new TrajectoryFollowerCommand(drive, traj2),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(drive, traj3)
                    .alongWith(new InstantCommand(() -> intake.set(1))),
                new TrajectoryFollowerCommand(drive, traj4)
                        .alongWith(new InstantCommand(() -> intake.set(1))),
                new InstantCommand(() -> intake.set(0)),
                new InstantCommand(() -> intake.set(-0.5)),
                new TrajectoryFollowerCommand(drive, traj0_1),
                new TurnCommand(drive, Math.toRadians(-100.0)).alongWith(new InstantCommand(drop::dropTwo).andThen(new InstantCommand(drop::dropThree))),
                new InstantCommand(() -> intake.set(0.5)),
                new TrajectoryFollowerCommand(drive, traj1)
                        .alongWith(new InstantCommand(drop::dropFour)),
                new ParallelCommandGroup(
                        new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)),
                        new InstantCommand(drop::dropOne)
                ),
                new TrajectoryFollowerCommand(drive, traj2)
                    .alongWith(new InstantCommand(() -> intake.set(-0.5))),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.set(1)).raceWith(new WaitCommand(1000))
        );

        // addCommands(
        //         new TrajectoryFollowerCommand(drive, traj0),
        //         new RunCommand(drop::initDrop).raceWith(new WaitCommand(1000)),
        //         new TrajectoryFollowerCommand(drive, traj1),
        //         new RunCommand(drop::halfDrop).raceWith(new WaitCommand(1000)),
        //         new RunCommand(drop::drop).raceWith(new WaitCommand(1000)),
        //         // new ParallelRaceGroup(
        //         //         new RunCommand(() -> intake.set(-0.5)).raceWith(new WaitCommand(3000)),
        //         //         new SequentialCommandGroup(
        //         //                 new TrajectoryFollowerCommand(drive, traj2),
        //         //                 new TrajectoryFollowerCommand(drive, traj3)
        //         //         )
        //         // ),
        //         new SequentialCommandGroup(
        //                 new TrajectoryFollowerCommand(drive, traj2),
        //                 new TrajectoryFollowerCommand(drive, traj3)
        //         ),
        //         // Intake
        //         // new ParallelDeadlineGroup(
        //         //         new SequentialCommandGroup(
        //         //                 new TrajectoryFollowerCommand(drive, traj1),
        //         //                 new TrajectoryFollowerCommand(drive, traj2),
        //         //                 new TrajectoryFollowerCommand(drive, traj3)
        //         //         ),
        //         //         new SequentialCommandGroup(
        //         //                 new RunCommand(drop::halfDrop).raceWith(new WaitCommand(1000)),
        //         //                 new RunCommand(drop::drop).raceWith(new WaitCommand(1000)),
        //         //                 new RunCommand(() -> intake.set(-0.5)).raceWith(new WaitCommand(3000))
        //         //         )
        //         // ),
        //         new TrajectoryFollowerCommand(drive, traj0),
        //         new RunCommand(drop::miniDrop).raceWith(new WaitCommand(1000)),
        //         new RunCommand(drop::initDrop).raceWith(new WaitCommand(1000))
        // );
    }

    public RedIntakePath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake, LiftSubsystemNoPID lift, int placement) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-6.0, 34.0, Math.toRadians(60.0))) // Drop init freight
//                .lineToLinearHeading(new Pose2d(0.0, 30.0, Math.toRadians(60.0))) // Drop init freight
//                .lineToLinearHeading(new Pose2d(6.0, 30.0, Math.toRadians(60.0))) // Drop init freight
                .lineToLinearHeading(new Pose2d(7.0, -24.0, Math.toRadians(-60.0))) // Drop init freight
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(8.0, -62.0, 0.0)) // Go back to start pos
                .build();

        Trajectory traj1_1 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(30)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // Go into warehouse
                .forward(50)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(30)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToLinearHeading(new Pose2d(9.0, 62.0, 0.0)) // Exit warehouse
//                .lineToLinearHeading(new Pose2d(9.0, 80.0, 0.0)) // Exit warehouse
                .back(50)
                .build();

        Trajectory traj0_1 = drive.trajectoryBuilder(traj4.end())
//                .lineToLinearHeading(new Pose2d(9.0, 58.0, Math.toRadians(0.0))) // Drop init freight after exiting warehouse
                .lineToLinearHeading(new Pose2d(9.0, -49.0, Math.toRadians(0.0))) // Drop init freight after exiting warehouse
                .build();

        addCommands(
                /* Drop Preloaded Freight */
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                new TrajectoryFollowerCommand(drive, traj0) // Go to team hub
                        .alongWith(new InstantCommand(drop::dropTwo)),
                new TurnCommand(drive, Math.toRadians(-20)),
                // TODO: Test if lifts work here
                new SelectCommand(new HashMap<Object, Command>(){{
                    put(2,
                            new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(1,
                            new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(0,
                            new WaitCommand(1200));
                }}, () -> placement)
                        .andThen(new InstantCommand(drop::dropThree).alongWith(new WaitCommand(1000)))
                        .andThen(new InstantCommand(drop::dropTwo).alongWith(new WaitCommand(1000)))
                        // Drop freight
                        // TODO: Test if lifts work here also
                        .andThen(new SelectCommand(new HashMap<Object, Command>(){{
                            put(2,
                                    new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))));
                            put(1,
                                    new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))));
                            put(0,
                                    new WaitCommand(200));
                        }}, () -> placement)),
                new TurnCommand(drive, Math.toRadians(50)),

                /* Intake freight from warehouse */
                // new InstantCommand(() -> intake.set(0.5)), // Idk why this is here
//                new TrajectoryFollowerCommand(drive, traj1), // Come back to start pos
                new TrajectoryFollowerCommand(drive, traj1_1),
                new RunCommand(() -> intake.set(-1)).raceWith(new WaitCommand(1000)).alongWith(new InstantCommand(drop::dropOne)),
//                new ParallelCommandGroup( // Stop intake to allow for carriage to go to starting pos for intaking
//                        new RunCommand(() -> intake.set(-1)).raceWith(new WaitCommand(1000)),
//                        new InstantCommand(drop::dropOne)
//                ),
                new InstantCommand(() -> intake.set(-0.45)), // Start intake for intaking
                new TrajectoryFollowerCommand(drive, traj2), // Go inside warehouse
                new WaitCommand(1000), // Stall to allow for blocks to be taken in
                new TrajectoryFollowerCommand(drive, traj3) // Strafe to left to adjust bot
                        .alongWith(new InstantCommand(() -> intake.set(1))), // Rapidly do outtake to eject extra freight
                new TrajectoryFollowerCommand(drive, traj4) // Go back to orig pos
                        .alongWith(new InstantCommand(() -> intake.set(1))), // Same as alongWith line above, not really needed
                new InstantCommand(() -> intake.set(0)), // This is useless

                /* Drop new freight */
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                new TrajectoryFollowerCommand(drive, traj0_1), // Go to shared hub
                new TurnCommand(drive, Math.toRadians(100.0)) // Turn back towards shared hub
                        .alongWith(new InstantCommand(drop::dropTwo) // Get ready to drop freight
                                .andThen(new InstantCommand(drop::dropThree))), // Drop freight
                new InstantCommand(() -> intake.set(0.5)), // Outtake to allow for carriage to go in
                new TrajectoryFollowerCommand(drive, traj1) // Go back to orig pos
                        .alongWith(new InstantCommand(drop::dropFour)), // Move to pos after dropping (not needed)
                new ParallelCommandGroup( // Stop intake to allow for carriage to go to starting pos for intaking
                        new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)),
                        new InstantCommand(drop::dropOne)
                ),
                new TrajectoryFollowerCommand(drive, traj2) // Go into warehouse for parking
                        .alongWith(new InstantCommand(() -> intake.set(-0.5))), // Runs intake to never take in freight
                new WaitCommand(1000), // Stall time
                new InstantCommand(() -> intake.set(1)).raceWith(new WaitCommand(1000)) // Runs outtake for rest of auton just in case we take in extra freight
        );

    }
}