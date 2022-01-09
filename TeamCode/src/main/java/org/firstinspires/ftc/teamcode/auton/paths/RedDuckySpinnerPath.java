package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DuckySpinnerCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Config
public class RedDuckySpinnerPath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(-36.0, -63.0, Math.toRadians(180.0));
    private Pose2d pose2 = new Pose2d(-60.0, -21.0, Math.toRadians(180.0));

    public RedDuckySpinnerPath(MecanumDriveSubsystem drive, DuckySpinnerSubsystem duckySpinnerSubsystem) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .forward(24.0)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(pose2)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(pose2)
                .forward(30.0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(32.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(15.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new DuckySpinnerCommand(duckySpinnerSubsystem, 0.25).withTimeout(5000),
                new TrajectoryFollowerCommand(drive, traj1),
                new TrajectoryFollowerCommand(drive, traj2),

                //TODO: Add outtake command here
                new TrajectoryFollowerCommand(drive, traj3),
                new TrajectoryFollowerCommand(drive, traj4)
        );
    }
}