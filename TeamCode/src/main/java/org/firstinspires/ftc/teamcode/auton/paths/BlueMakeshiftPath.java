package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.Arrays;

import javax.annotation.Nullable;

public class BlueMakeshiftPath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(-12.0, 62.0, Math.toRadians(90.0));
    private Pose2d pose2 = new Pose2d(-12.0, 44.0, Math.toRadians(10.0));

    public BlueMakeshiftPath(MecanumDriveSubsystem drive, DropSubsystem sDrop, Motor mIntake, @Nullable ElapsedTime time) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(25) //18, 25
                .build();

        // Trajectory traj1 = drive.trajectoryBuilder(pose2)
        //         .forward(65.0,
        //                 new MinVelocityConstraint(Arrays.asList(
        //                         new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
        //                         new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
        //                 )),
        //                 new ProfileAccelerationConstraint(105.0)
        //                 ) // 57
        //         .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .forward(50.0)
                .build();

        // addCommands(
        //         new TrajectoryFollowerCommand(drive, traj0),
        //         new RunCommand(sDrop::initDrop).raceWith(new WaitCommand(2000)),
        //         new RunCommand(sDrop::halfDrop).raceWith(new WaitCommand(2000)),
        //         new RunCommand(sDrop::drop).raceWith(new WaitCommand(2000)),
        //         // new TurnCommand(drive, Math.toRadians(-200)),
        //         new TurnCommand(drive, Math.toRadians(210)),
        //         new TrajectoryFollowerCommand(drive, traj1)//.alongWith(new StartEndCommand(() -> mIntake.set(0.5), () -> mIntake.stopMotor()))
        // );

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new RunCommand(sDrop::initDrop).raceWith(new WaitCommand(1000)),
                new ParallelCommandGroup(
                        new TurnCommand(drive, Math.toRadians(197)).andThen(new TrajectoryFollowerCommand(drive, traj1)),
                        new RunCommand(sDrop::halfDrop).raceWith(new WaitCommand(1000)).andThen(new RunCommand(sDrop::drop).raceWith(new WaitCommand(1000)).andThen(new RunCommand(() -> mIntake.set(-0.5)).raceWith(new WaitCommand(5000)).andThen(new InstantCommand(() -> mIntake.set(0)))))
                )
                // new TurnCommand(drive, Math.toRadians(-200)),
        );
    }
}
