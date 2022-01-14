package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RedIntakePath extends SequentialCommandGroup {
    private Pose2d startPose = new Pose2d(9.0, -62.0, 0.0);

    public RedIntakePath(MecanumDriveSubsystem drive, DropSubsystem drop, Motor intake) {
        drive.setPoseEstimate(startPose);

        // Goes from start position to shipping hub
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(9.0, -57.0, Math.toRadians(-60.0)))
                .back(23.0)
                .build();

        // Goes from shipping hub to start position
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .forward(23.0)
                .lineToLinearHeading(startPose)
                .build();

        // Start to intake location
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(36.0)
                .build();

        // Intake motion
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(12.0)
                .back(12.0)
                .build();

        // Intake location to start
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(36.0)
                .build();

        addCommands(
                // Initial drop
                new TrajectoryFollowerCommand(drive, traj0),
                new RunCommand(drop::initDrop).raceWith(new WaitCommand(500)),
                // First cycle
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RunCommand(drop::halfDrop).raceWith(new WaitCommand(500)),
                                new RunCommand(drop::drop).raceWith(new WaitCommand(500))
                        ),
                        new TrajectoryFollowerCommand(drive, traj1)
                ),
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, traj2),
                            new WaitCommand(250),
                            new TrajectoryFollowerCommand(drive, traj3),
                            new TrajectoryFollowerCommand(drive, traj4),
                            new ParallelCommandGroup(
                                    new TrajectoryFollowerCommand(drive, traj0),
                                    new RunCommand(drop::miniDrop).raceWith(new WaitCommand(500))
                            )
                    ),
                    new StartEndCommand(() -> intake.set(0.5), () -> intake.set(0))
                        //hahahahah lamdasssss
                ),
                new RunCommand(drop::initDrop).raceWith(new WaitCommand(500))
        );
    }
}
