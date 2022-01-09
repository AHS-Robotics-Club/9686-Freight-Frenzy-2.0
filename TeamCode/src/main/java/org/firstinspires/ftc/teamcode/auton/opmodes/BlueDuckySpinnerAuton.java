package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.paths.BlueDuckySpinnerPath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

// TODO: Add intake subsystem
public class BlueDuckySpinnerAuton extends CommandOpMode {

    // Motors
    private Motor frontLeft, frontRight, backLeft, backRight;
    private Motor mDuckySpinner;

    // Subsystems
    private MecanumDriveSubsystem drive;
    private DuckySpinnerSubsystem duckySpinner;

    // Timed Action
    private ElapsedTime time;

    @Override
    public void initialize() {

        mDuckySpinner = new Motor(hardwareMap, "duckSpinner");
        mDuckySpinner.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mDuckySpinner.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        duckySpinner = new DuckySpinnerSubsystem(mDuckySpinner);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        SequentialCommandGroup autonomous = new BlueDuckySpinnerPath(drive, duckySpinner);

        schedule(autonomous);
    }
}
