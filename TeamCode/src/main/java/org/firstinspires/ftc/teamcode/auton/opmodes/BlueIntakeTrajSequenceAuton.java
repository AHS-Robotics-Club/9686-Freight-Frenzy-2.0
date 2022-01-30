package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auton.paths.BlueIntakePath;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueIntakeAuton-TrajectorySequence")
public class BlueIntakeTrajSequenceAuton extends CommandOpMode {
    private SimpleServo dropLeft, dropRight;
    private Motor intake;

    private SampleMecanumDrive drive;
    private DropSubsystem drop;
    private LiftSubsystemNoPID lift;

    private Pose2d startPose;

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);

        dropLeft = new SimpleServo(hardwareMap, "leftDrop", -90, 90);
        dropRight = new SimpleServo(hardwareMap, "rightDrop", -90, 90);
        intake = new Motor(hardwareMap, "intake");

        startPose = new Pose2d(9.0, 62.0, 0.0);
        drop = new DropSubsystem(dropLeft, dropRight);

        lift = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "leftLift"),
                new Motor(hardwareMap, "leftLift")
        );

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> intake.set(-0.5))
                .lineToLinearHeading(new Pose2d(6.0, 24.0, Math.toRadians(60.0)))   // TODO | Trajectory 1: Go to team hub
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))))
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

        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500)).andThen(new RunCommand(() -> drive.followTrajectorySequence(trajectorySequence))));

    }

//        schedule(auton.andThen(auton1));
}