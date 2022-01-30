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
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.time.Instant;
import java.util.HashMap;

@Config
public class RedCarouselPath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(-28.0, -62.0, Math.toRadians(90.0));

    public static double DISTANCE = 24.0;
    public static double TURN = 20;

    public static double DISTANCE0 = 12.5;
    public static double TURN0 = 210;
    public static double TURN1 = -45;
    public static double DISTANCE1 = 22.0;

    public RedCarouselPath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake, Motor duckySpinner) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose) // Drop init freight
//                .lineToLinearHeading(new Pose2d(-6.0, 34.0, Math.toRadians(60.0)))
//                .lineToLinearHeading(new Pose2d(0.0, 30.0, Math.toRadians(60.0)))
//                .lineToLinearHeading(new Pose2d(6.0, 30.0, Math.toRadians(60.0)))

               // .lineToLinearHeading(new Pose2d(-20.0, -28.0, Math.toRadians(260.0)))
                //.build();
                    .forward(43.0)
                    .build();

        Trajectory traj0_1 = drive.trajectoryBuilder(traj0.end())
                .back(20)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-28.0, -62.0, Math.toRadians(0))) // Go back to start pos from team hub
                .forward(52.0)
                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // Go into warehouse
//                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(180.0))) // Park
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()) // Realign against wall
//                .forward(37)
//                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(5)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .back(24.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()) // Exit warehouse
                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(0))) // Park
                .build();


        addCommands(
                new InstantCommand(() -> intake.set(-0.5)),       // Set intake on to pick up any items
                new TrajectoryFollowerCommand(drive, traj0)// Go to Alliance Hub
                        .alongWith(new InstantCommand(drop::dropTwo)),    // Set carriage parallel to ground so it doesn't fall out
//                                .andThen(new WaitCommand(1200)          // Wait so that block doesn't get launched
//                                        .andThen(new InstantCommand(drop::dropThree)))),
                new TurnCommand(drive, Math.toRadians(180)),
                new TrajectoryFollowerCommand(drive, traj0_1),
                new InstantCommand(drop::dropThree).alongWith(new WaitCommand(1250)), // Bring down carriage

                new InstantCommand(() -> intake.set(-1)).alongWith(new InstantCommand(drop::dropOne).alongWith(new WaitCommand(1000))),                   // Set intake to pick up any items
                new TurnCommand(drive, Math.toRadians(80.0)),
                new TrajectoryFollowerCommand(drive, traj1)                   // Go to the carousel
                        .andThen(new RunCommand(() -> duckySpinner.set(0.4)) // Run ducky spinner motor after going there
                                .raceWith(new WaitCommand(2950))),          // Wait till the duck drops
                new TrajectoryFollowerCommand(drive, traj2),
                new TurnCommand(drive, Math.toRadians(100)),// Go to parking
                new TrajectoryFollowerCommand(drive, traj3)
        );
        //region COLLAPSE
                /* Drop Preloaded Freight
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                new TrajectoryFollowerCommand(drive, traj0) // Go to team hub
                        .alongWith(new InstantCommand(drop::dropTwo) // TODO Pick up intake
                        .andThen(new WaitCommand(1200) // Stall time so no launch
                        .andThen(new InstantCommand(drop::dropThree)))), // Drop freight

                // Intake freight from warehouse
                new InstantCommand(() -> intake.set(0.5)), // Idk why this is here
                new TrajectoryFollowerCommand(drive, traj1) // Come back to start pos
                    .alongWith(new InstantCommand(drop::dropFour)), // Move to pos after dropping (not needed)
                new ParallelCommandGroup( // Stop intake to allow for carriage to go to starting pos for intaking
                    new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)),
                    new InstantCommand(drop::dropOne)
                ),
                new InstantCommand(() -> intake.set(-0.45)), // Start intake for intaking
                new TrajectoryFollowerCommand(drive, traj2), // Go inside warehouse
                new WaitCommand(1000), // Stall to allow for blocks to be taken in
                new TrajectoryFollowerCommand(drive, traj3) // Strafe to left to adjust bot
                    .alongWith(new InstantCommand(() -> intake.set(1))), // Rapidly do outtake to eject extra freight
                new TrajectoryFollowerCommand(drive, traj4) // Go back to orig pos
                    .alongWith(new InstantCommand(() -> intake.set(1))), // Same as alongWith line above, not really needed
                new InstantCommand(() -> intake.set(0)), // This is useless

                // Drop new freight
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                //new TrajectoryFollowerCommand(drive, traj0_1), // Go to shared hub
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
                 */
        //endregion
    }



    public RedCarouselPath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake, Motor duckySpinner, LiftSubsystemNoPID lift, int placement) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose) // Drop init freight
//                .lineToLinearHeading(new Pose2d(-6.0, 34.0, Math.toRadians(60.0)))
//                .lineToLinearHeading(new Pose2d(0.0, 30.0, Math.toRadians(60.0)))
//                .lineToLinearHeading(new Pose2d(6.0, 30.0, Math.toRadians(60.0)))

                // .lineToLinearHeading(new Pose2d(-20.0, -28.0, Math.toRadians(260.0)))
                //.build();
                .forward(DISTANCE0)
                .build();

        Trajectory traj0_1 = drive.trajectoryBuilder(traj0.end())
                .back(14)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-28.0, -62.0, Math.toRadians(0))) // Go back to start pos from team hub
                .forward(DISTANCE1)
                .build();

        Trajectory traj1_0 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(16)
                .build();

        Trajectory traj1_1 = drive.trajectoryBuilder(traj1.end())
                .forward(15)
                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // Go into warehouse
//                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(180.0))) // Park
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()) // Realign against wall
//                .forward(37)
//                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(6)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .back(20.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()) // Exit warehouse
                .lineToLinearHeading(new Pose2d(-60.0, -35.5, Math.toRadians(0))) // Park
                .build();


        addCommands(
                new InstantCommand(() -> intake.set(-0.5)),       // Set intake on to pick up any items
                new TrajectoryFollowerCommand(drive, traj0)// Go to Alliance Hub
                        .alongWith(new InstantCommand(drop::dropTwo)),    // Set carriage parallel to ground so it doesn't fall out
//                                .andThen(new WaitCommand(1200)          // Wait so that block doesn't get launched
//                                        .andThen(new InstantCommand(drop::dropThree)))),
                new TurnCommand(drive, Math.toRadians(TURN0)),
                new TrajectoryFollowerCommand(drive, traj0_1),
                // TODO: Test if lifts work here
                new SelectCommand(new HashMap<Object, Command>(){{
                    put(2,
                            new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(1,
                            new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(0,
                            new WaitCommand(1200));
                }}, () -> placement)
                .andThen(new InstantCommand(drop::dropThree).alongWith(new WaitCommand(1000))) // Drop freight
                // TODO: Test if lifts work here also
                .andThen(new SelectCommand(new HashMap<Object, Command>(){{
                    put(2,
                            new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(1,
                            new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))));
                    put(0,
                            new WaitCommand(200));
                }}, () -> placement)),

                new InstantCommand(() -> intake.set(-1)).alongWith(new InstantCommand(drop::dropOne).alongWith(new WaitCommand(1000))),                   // Set intake to pick up any items
                new TurnCommand(drive, Math.toRadians(TURN1)),
                new TrajectoryFollowerCommand(drive, traj1),
                new TrajectoryFollowerCommand(drive, traj1_0),
                new TurnCommand(drive, Math.toRadians(60)),
                new TrajectoryFollowerCommand(drive, traj1_1)// Go to the carousel
                        .andThen(new RunCommand(() -> duckySpinner.set(0.4)) // Run ducky spinner motor after going there
                                .raceWith(new WaitCommand(2950))),          // Wait till the duck drops
                new TrajectoryFollowerCommand(drive, traj2),
                new TurnCommand(drive, Math.toRadians(100)),// Go to parking
                new TrajectoryFollowerCommand(drive, traj3)
        );
        //region COLLAPSE
                /* Drop Preloaded Freight
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                new TrajectoryFollowerCommand(drive, traj0) // Go to team hub
                        .alongWith(new InstantCommand(drop::dropTwo) // TODO Pick up intake
                        .andThen(new WaitCommand(1200) // Stall time so no launch
                        .andThen(new InstantCommand(drop::dropThree)))), // Drop freight

                // Intake freight from warehouse
                new InstantCommand(() -> intake.set(0.5)), // Idk why this is here
                new TrajectoryFollowerCommand(drive, traj1) // Come back to start pos
                    .alongWith(new InstantCommand(drop::dropFour)), // Move to pos after dropping (not needed)
                new ParallelCommandGroup( // Stop intake to allow for carriage to go to starting pos for intaking
                    new RunCommand(() -> intake.set(0)).raceWith(new WaitCommand(1000)),
                    new InstantCommand(drop::dropOne)
                ),
                new InstantCommand(() -> intake.set(-0.45)), // Start intake for intaking
                new TrajectoryFollowerCommand(drive, traj2), // Go inside warehouse
                new WaitCommand(1000), // Stall to allow for blocks to be taken in
                new TrajectoryFollowerCommand(drive, traj3) // Strafe to left to adjust bot
                    .alongWith(new InstantCommand(() -> intake.set(1))), // Rapidly do outtake to eject extra freight
                new TrajectoryFollowerCommand(drive, traj4) // Go back to orig pos
                    .alongWith(new InstantCommand(() -> intake.set(1))), // Same as alongWith line above, not really needed
                new InstantCommand(() -> intake.set(0)), // This is useless

                // Drop new freight
                new InstantCommand(() -> intake.set(-0.5)), // Start intake to take out carriage
                //new TrajectoryFollowerCommand(drive, traj0_1), // Go to shared hub
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
                 */
        //endregion
    }
}