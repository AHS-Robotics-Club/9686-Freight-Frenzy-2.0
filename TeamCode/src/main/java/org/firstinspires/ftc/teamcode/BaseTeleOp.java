package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "BaseTeleOp")
public class BaseTeleOp extends CommandOpMode {

    private final boolean servos = false;

    // Motors
    private Motor frontLeft, frontRight, backLeft, backRight; // Drivetrain
    // private Motor mDuckySpinner; // Other motors

    // Extra Servo Stuff
    private SimpleServo sLeftDrop, sRightDrop;
    private DropSubsystem dropS;
    private DropCommand drop_Com;

    // Subsystems
    private DriveSubsystem driveS;
    private MecanumDriveSubsystem mecanumDriveS;
    // private DuckySpinnerSubsystem duckySpinnerS;

    // Commands
    private DriveCommand drive_Com;
    // private DuckySpinnerCommand duckySpinner_Com;

    // Extra Stuff
    private GamepadEx gPad1;
    private FtcDashboard dashboard;
    private RevIMU revIMU;

    // Commands Stuff

    // Constants
    private final double DRIVE_MULT = 1.0;
    private final double SLOW_MULT = 0.3;
    private final double BLUE_DS = 0.25;
    private final double RED_DS = -0.25;

    @Override
    public void initialize() {

        // Telemetry for Dashboard
        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drivetrain Motors
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");


        // Extra Stuff Setup
        revIMU = new RevIMU(hardwareMap);
        revIMU.init();

        gPad1 = new GamepadEx(gamepad1);

        dashboard = FtcDashboard.getInstance();

        // Subsystems & Commands
        mecanumDriveS = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        // frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        // backRight.motor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveS = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight, revIMU);
        drive_Com = new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX);

        // Optional Servos
        if(servos) {
            sLeftDrop = new SimpleServo(hardwareMap, "leftDrop", 0, 180);
            sRightDrop = new SimpleServo(hardwareMap, "rightDrop", 0, 180);

            dropS = new DropSubsystem(sLeftDrop, sRightDrop);
            drop_Com = new DropCommand(dropS);

            gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(drop_Com);
        }

        // Slow Drivetrain
        gPad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                () -> new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, SLOW_MULT),
                () -> new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, DRIVE_MULT)
        );

        // Sets default command for drivetrain
        register(driveS);
        driveS.setDefaultCommand(drive_Com);

        // schedule(new InitDropCommand(sLeftDrop, sRightDrop));

        // How to add telemetry, not that relevant rn
        schedule(new RunCommand(() -> {
            if(servos) {
                telemetry.addData("Left Servo Angle", sLeftDrop.getAngle());
                telemetry.addData("Right Servo Angle", sRightDrop.getAngle());
            }
            telemetry.addData("frontLeft encoder position", frontLeft.encoder.getPosition());
            telemetry.addData("frontRight encoder position", frontRight.encoder.getPosition());
            telemetry.addData("backLeft encoder position", backLeft.encoder.getPosition());
            telemetry.addData("backRight encoder position", backRight.encoder.getPosition());
            telemetry.update();
        }));
    }
}
