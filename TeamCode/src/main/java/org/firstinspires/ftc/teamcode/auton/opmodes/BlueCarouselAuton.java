package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auton.paths.BlueIntakePath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name="BlueCarouselAuton")
public class BlueCarouselAuton extends CommandOpMode{
    private Motor mDuckySpinner;
    private Motor mIntake;
    private Motor mRightLift, mLeftLift;

    private SimpleServo sLeftDrop, sRightDrop;

    private MecanumDriveSubsystem drive;
    private DuckySpinnerSubsystem duckySpinnerSubsystem;
    private DropSubsystem dropSubsystem;

    @Override
    public void initialize() {
        mDuckySpinner = new Motor(hardwareMap, "duckySpinner");
        mIntake = new Motor(hardwareMap, "intake");

        mRightLift = new Motor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435);
        mLeftLift = new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435);

        sLeftDrop = new SimpleServo(hardwareMap, "leftDrop", -180, 180);
        sRightDrop = new SimpleServo(hardwareMap, "rightDrop", -180, 180);

        dropSubsystem = new DropSubsystem(sLeftDrop, sRightDrop);
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        duckySpinnerSubsystem = new DuckySpinnerSubsystem(mDuckySpinner);

        SequentialCommandGroup auton = new BlueIntakePath(drive, dropSubsystem, mIntake);


        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500)).andThen(auton));
    }
}
