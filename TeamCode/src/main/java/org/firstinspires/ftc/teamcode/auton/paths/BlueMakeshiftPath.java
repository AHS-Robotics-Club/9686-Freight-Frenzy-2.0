package org.firstinspires.ftc.teamcode.auton.paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import javax.annotation.Nullable;

@Config
public class BlueMakeshiftPath extends SequentialCommandGroup {

    private Pose2d startPose = new Pose2d(-12.0, 62.0, Math.toRadians(90.0));
    private Pose2d pose2 = new Pose2d(-12.0, 44.0, Math.toRadians(10.0));

    public static double BACK = 17;
    public static double TURN = 160;
    public static double FORWARD = 35.0;

    public BlueMakeshiftPath(MecanumDriveSubsystem drive, DropSubsystem sDrop, Motor mIntake, Motor dS, @Nullable ElapsedTime time) {
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(BACK) //18, 25
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
                .forward(FORWARD)
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
                new RunCommand(sDrop::dropThree).raceWith(new WaitCommand(1000)),
                new ParallelCommandGroup(
                        new TurnCommand(drive, Math.toRadians(TURN)).andThen(new TrajectoryFollowerCommand(drive, traj1)),
                        new RunCommand(sDrop::dropFour).raceWith(new WaitCommand(1000)).andThen(new RunCommand(sDrop::dropOne).raceWith(new WaitCommand(1000)).andThen(new RunCommand(() -> mIntake.set(-0.5)).raceWith(new WaitCommand(5000)).andThen(new InstantCommand(() -> mIntake.set(0)))))
                )
//                new TurnCommand(drive, Math.toRadians(360 - TURN)),
//                new TrajectoryFollowerCommand(drive, traj1)
//                .andThen(new RunCommand(() -> dS.set(0.4)) // Run ducky spinner motor after going there
//                .raceWith(new WaitCommand(2950)))
        // new TurnCommand(drive, Math.toRadians(-200)),
        );
    }
}
