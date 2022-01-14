package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class BlueIntakePath extends SequentialCommandGroup {
    private Pose2d startPose = new Pose2d(9.0, 62.0, 0.0);

    public BlueIntakePath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake) {
        drive.setPoseEstimate(startPose);

        // Goes from start position to shipping hub
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                // 60.0
                .lineToLinearHeading(new Pose2d(9.0, 57.0, Math.toRadians(90.0)))
//                .back(23.0)
                .build();

        Trajectory trajj0 = drive.trajectoryBuilder(traj0.end())
                .back(25.0)
                .build();

        Trajectory trajj1 = drive.trajectoryBuilder(trajj0.end())
                .forward(25.0)
                .build();

        // Goes from shipping hub to start position
        Trajectory traj1 = drive.trajectoryBuilder(trajj1.end())
//        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
//                .forward(23.0)
                .lineToLinearHeading(new Pose2d(9.0, 64.0, 0.0))
                .build();

        // Start to intake location
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(9.0, 65.0, 0.0))
                .forward(40.0)
                .build();

        // Intake motion
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(20.0)
//                .back(12.0)
                .build();

        Trajectory trajj3 = drive.trajectoryBuilder(traj3.end())
                .back(20.0)
                .build();

        Trajectory trajjj3 = drive.trajectoryBuilder(trajj3.end())
                .strafeLeft(25.0)
                .build();

        // Intake location to start
        Trajectory traj4 = drive.trajectoryBuilder(trajjj3.end())
                .back(40.0)
                .build();

        addCommands(
                // Initial drop
                new TrajectoryFollowerCommand(drive, traj0),
                new TrajectoryFollowerCommand(drive, trajj0),
                new RunCommand(drop::initDrop).raceWith(new WaitCommand(1000)),
                // First cycle
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RunCommand(drop::halfDrop).raceWith(new WaitCommand(1000)),
                                new RunCommand(drop::drop).raceWith(new WaitCommand(1000))
                        ),
                        new SequentialCommandGroup(
                                new TrajectoryFollowerCommand(drive, trajj1),
                                new TrajectoryFollowerCommand(drive, traj1)
                        )
                ),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, traj2),
                            new WaitCommand(250),
                            new TrajectoryFollowerCommand(drive, traj3),
                            new TrajectoryFollowerCommand(drive, trajj3),
                            new TrajectoryFollowerCommand(drive, trajjj3),
                            new TrajectoryFollowerCommand(drive, traj4),
                            new TrajectoryFollowerCommand(drive, traj0)
                    ),
                    // new StartEndCommand(() -> intake.set(-0.5), () -> intake.set(0))
                    new InstantCommand(() -> intake.set(-0.5))
                        //hahahahah lamdasssss
                ),
//                new InstantCommand(() -> intake.set(0)),
                new TrajectoryFollowerCommand(drive, traj0)
//                new RunCommand(drop::miniDrop).raceWith(new WaitCommand(500)),
                // new ParallelCommandGroup(
                //         new TrajectoryFollowerCommand(drive, trajj0),
                // ),
//                new TrajectoryFollowerCommand(drive, trajj0),
//                new RunCommand(drop::initDrop).raceWith(new WaitCommand(500))
        );
    }
}
