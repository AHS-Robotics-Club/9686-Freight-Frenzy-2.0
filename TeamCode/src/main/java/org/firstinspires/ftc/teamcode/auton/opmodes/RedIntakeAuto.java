package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.auton.paths.RedIntakePath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RedIntakeAuto extends CommandOpMode {

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

        SequentialCommandGroup auton = new RedIntakePath(drive, dropSubsystem, mIntake);

        schedule(auton);
    }
}
