package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auton.paths.BlueIntakePath;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "BlueIntakeAuto")
public class BlueIntakeAuto extends CommandOpMode {

    private SimpleServo sDropLeft, sDropRight;
    private Motor mIntake;

    private MecanumDriveSubsystem drive;
    private DropSubsystem dropSubsystem;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        sDropLeft = new SimpleServo(hardwareMap, "leftDrop", -90, 90);
        sDropRight = new SimpleServo(hardwareMap, "rightDrop", -90, 90);
        mIntake = new Motor(hardwareMap, "intake");

        dropSubsystem = new DropSubsystem(sDropLeft, sDropRight);

        SequentialCommandGroup auton = new BlueIntakePath(drive, dropSubsystem, mIntake);

        Trajectory trajj0 = drive.trajectoryBuilder(new Pose2d(9.0, 57.0, Math.toRadians(60.0)))
                .back(23.0)
                .build();

        SequentialCommandGroup auton1 = new SequentialCommandGroup(
//                new RunCommand(dropSubsystem::miniDrop).raceWith(new WaitCommand(500)),
                // new ParallelCommandGroup(
                //         new TrajectoryFollowerCommand(drive, trajj0),
                // ),
                new TrajectoryFollowerCommand(drive, trajj0)
//                new RunCommand(dropSubsystem::initDrop).raceWith(new WaitCommand(500))
        );

//        schedule(auton.andThen(auton1));
            schedule(auton.andThen(new TrajectoryFollowerCommand(drive, trajj0)));
    }
}
