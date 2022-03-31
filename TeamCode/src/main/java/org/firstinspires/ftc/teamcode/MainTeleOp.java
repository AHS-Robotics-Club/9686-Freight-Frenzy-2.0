package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.commands.SecureBlockCommand.isSecured;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.DropCommandInitLift;
import org.firstinspires.ftc.teamcode.commands.HighLiftCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPIDDown;
import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPIDFix;
import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPIDUp;
import org.firstinspires.ftc.teamcode.commands.SecureBlockCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends CommandOpMode {

     /*
    $$$$$$$\                      $$\                               $$\     $$\
    $$  __$$\                     $$ |                              $$ |    \__|
    $$ |  $$ | $$$$$$\   $$$$$$$\ $$ | $$$$$$\   $$$$$$\  $$$$$$\ $$$$$$\   $$\  $$$$$$\  $$$$$$$\   $$$$$$$\
    $$ |  $$ |$$  __$$\ $$  _____|$$ | \____$$\ $$  __$$\ \____$$\\_$$  _|  $$ |$$  __$$\ $$  __$$\ $$  _____|
    $$ |  $$ |$$$$$$$$ |$$ /      $$ | $$$$$$$ |$$ |  \__|$$$$$$$ | $$ |    $$ |$$ /  $$ |$$ |  $$ |\$$$$$$\
    $$ |  $$ |$$   ____|$$ |      $$ |$$  __$$ |$$ |     $$  __$$ | $$ |$$\ $$ |$$ |  $$ |$$ |  $$ | \____$$\
    $$$$$$$  |\$$$$$$$\ \$$$$$$$\ $$ |\$$$$$$$ |$$ |     \$$$$$$$ | \$$$$  |$$ |\$$$$$$  |$$ |  $$ |$$$$$$$  |
    \_______/  \_______| \_______|\__| \_______|\__|      \_______|  \____/ \__| \______/ \__|  \__|\_______/
     */

    // Motors
    private Motor frontLeft, frontRight, backLeft, backRight; // Drivetrain
    private Motor mDuckySpinner, mIntake, mLeftLift, mRightLift; // Other motors

    // Servos
    private SimpleServo sLeftDrop, sRightDrop;

    // Subsystems
    private DriveSubsystem driveS;
    private MecanumDriveSubsystem mecanumDriveS;
    private LiftSubsystemNoPID liftNoPIDS;
    private DropSubsystem dropS;

    // Commands
    private DriveCommand drive_Com;
    // private LiftCommandNoPID liftNoPID_Com;
    private LiftCommandNoPIDUp liftNoPIDUp_Com;
    private LiftCommandNoPIDDown liftNoPIDDown_Com;
    private LiftCommandNoPIDFix liftNoPIDFixUp_Com;
    private LiftCommandNoPIDFix liftNoPIDFixDown_Com;
    private DropCommand drop_Com;
    private DropCommandInitLift dropInitLift_Com;

    private SecureBlockCommand secureBlock_Com;
    private HighLiftCommand highLift_Com;

    // Extra Stuff
    private GamepadEx gPad1;
    private FtcDashboard dashboard;
    private ElapsedTime time;
    private RevIMU revIMU;

    // Commands Stuff

    // Constants
    private final double DRIVE_MULT = 1.0;
    private final double SLOW_MULT = 0.5;

    private final double INTAKE_MULT = 0.45;
    private final double OUTTAKE_MULT = -0.45;

    private final double BLUE_DS = 0.45;
    private final double RED_DS = -0.45;

    // Other Stuff
    public static boolean isHigh = false;
    public static boolean wasHigh = false;

    @Override
    public void initialize() {

        /*
         /$$$$$$           /$$   /$$     /$$           /$$ /$$                       /$$     /$$
        |_  $$_/          |__/  | $$    |__/          | $$|__/                      | $$    |__/
          | $$   /$$$$$$$  /$$ /$$$$$$   /$$  /$$$$$$ | $$ /$$ /$$$$$$$$  /$$$$$$  /$$$$$$   /$$  /$$$$$$  /$$$$$$$   /$$$$$$$
          | $$  | $$__  $$| $$|_  $$_/  | $$ |____  $$| $$| $$|____ /$$/ |____  $$|_  $$_/  | $$ /$$__  $$| $$__  $$ /$$_____/
          | $$  | $$  \ $$| $$  | $$    | $$  /$$$$$$$| $$| $$   /$$$$/   /$$$$$$$  | $$    | $$| $$  \ $$| $$  \ $$|  $$$$$$
          | $$  | $$  | $$| $$  | $$ /$$| $$ /$$__  $$| $$| $$  /$$__/   /$$__  $$  | $$ /$$| $$| $$  | $$| $$  | $$ \____  $$
         /$$$$$$| $$  | $$| $$  |  $$$$/| $$|  $$$$$$$| $$| $$ /$$$$$$$$|  $$$$$$$  |  $$$$/| $$|  $$$$$$/| $$  | $$ /$$$$$$$/
        |______/|__/  |__/|__/   \___/  |__/ \_______/|__/|__/|________/ \_______/   \___/  |__/ \______/ |__/  |__/|_______/
        */

        // Telemetry for Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors
        // drivetrain
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        // other
        mDuckySpinner = new Motor(hardwareMap, "duckySpinner");
        mIntake = new Motor(hardwareMap, "intake");
        mLeftLift = new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435);
        mRightLift = new Motor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435);

        // Servos
        sLeftDrop = new SimpleServo(hardwareMap, "leftDrop", -180, 180);
        sRightDrop = new SimpleServo(hardwareMap, "rightDrop", -180, 180);

        // Extra Stuff Setup
        revIMU = new RevIMU(hardwareMap);
        revIMU.init();
        gPad1 = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();
        time = new ElapsedTime();

        // Subsystems & Commands
        mecanumDriveS = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftNoPIDS = new LiftSubsystemNoPID(mLeftLift, mRightLift);

        driveS = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight, revIMU);
        drive_Com = new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, DRIVE_MULT);

        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        // frontRight.motor.setDirection(DcMotor.Direction.FORWARD); // TODO: Uncomment later
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        // backRight.motor.setDirection(DcMotor.Direction.FORWARD);

        // frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropS = new DropSubsystem(sLeftDrop, sRightDrop);
        drop_Com = new DropCommand(dropS);
        dropInitLift_Com = new DropCommandInitLift(dropS, liftNoPIDS, time);

        liftNoPIDS = new LiftSubsystemNoPID(mLeftLift, mRightLift);
        // liftNoPID_Com = new LiftCommandNoPID(liftNoPIDS, time, sLeftDrop, sRightDrop);
        liftNoPIDUp_Com = new LiftCommandNoPIDUp(liftNoPIDS, time);
        liftNoPIDDown_Com = new LiftCommandNoPIDDown(liftNoPIDS, time);
        liftNoPIDFixUp_Com = new LiftCommandNoPIDFix(liftNoPIDS, time, true);
        liftNoPIDFixDown_Com = new LiftCommandNoPIDFix(liftNoPIDS, time, false);

        secureBlock_Com = new SecureBlockCommand(liftNoPIDS, dropS, time);
        highLift_Com = new HighLiftCommand(liftNoPIDS, dropS, time);

        /*
        ▒█▀▀█ █▀▀█ █▀▄▀█ █▀▀ █▀▀█ █▀▀█ █▀▀▄
        ▒█░▄▄ █▄▄█ █░▀░█ █▀▀ █░░█ █▄▄█ █░░█
        ▒█▄▄█ ▀░░▀ ▀░░░▀ ▀▀▀ █▀▀▀ ▀░░▀ ▀▀▀░
         */

        // Ducky Spinner, no subsystem or command needed
        gPad1.getGamepadButton(GamepadKeys.Button.X).whenHeld(new StartEndCommand(() -> mDuckySpinner.set(BLUE_DS), () -> mDuckySpinner.stopMotor()));
        gPad1.getGamepadButton(GamepadKeys.Button.B).whenHeld(new StartEndCommand(() -> mDuckySpinner.set(RED_DS), () -> mDuckySpinner.stopMotor()));

        // Intake
        gPad1.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new StartEndCommand(() -> mIntake.set(INTAKE_MULT), () -> mIntake.stopMotor()));
        gPad1.getGamepadButton(GamepadKeys.Button.A).whenHeld(new StartEndCommand(() -> mIntake.set(OUTTAKE_MULT), () -> mIntake.stopMotor()));

        // Lift
        // gPad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(liftNoPID_Com);
        // gPad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(liftNoPIDUp_Com);
        // gPad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(liftNoPIDDown_Com);
        gPad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(highLift_Com); // TODO: Test high lift command

        gPad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(liftNoPIDFixDown_Com);
        gPad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(liftNoPIDFixUp_Com);

        // Drop
        // gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(drop_Com);
        // gPad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).toggleWhenPressed(drop_Com);
        gPad1.getGamepadButton(GamepadKeys.Button.BACK).toggleWhenPressed(new InstantCommand(dropS::dropThree), new InstantCommand(dropS::dropTwo));

        gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(secureBlock_Com); // TODO: Test new carriage


        // Slow Drivetrain
        // gPad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new StartEndCommand(
        //         () -> new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, SLOW_MULT),
        //         () -> new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, DRIVE_MULT)
        // ));

        gPad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new DriveCommand(driveS, gPad1::getLeftX, gPad1::getLeftY, gPad1::getRightX, SLOW_MULT));

        // Temporarily adding SecureBlockCommand
        // TODO: This may not work
        // gPad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(secureBlock_Com);
        // gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(highLift_Com);

        // Sets default command for drivetrain
        register(driveS);
        driveS.setDefaultCommand(drive_Com);

        // How to add telemetry, not that relevant rn
        schedule(new RunCommand(() -> {
            telemetry.addData("Left Servo Angle", sLeftDrop.getAngle());
            telemetry.addData("Right Servo Angle", sRightDrop.getAngle());
            telemetry.addData("Left Servo Position", sLeftDrop.getPosition());
            telemetry.addData("Right Servo Position", sRightDrop.getPosition());
            telemetry.addData("Left Lift Motor Speed", mLeftLift.getCorrectedVelocity());
            telemetry.addData("Right Lift Motor Speed", mRightLift.getCorrectedVelocity());
            telemetry.addData("Intake Motor Speed", mIntake.getCorrectedVelocity());
            telemetry.addData("Left Lift Motor Feedforward", mLeftLift.getFeedforwardCoefficients());
            telemetry.addData("Right Lift Motor Feedforward", mRightLift.getFeedforwardCoefficients());
            telemetry.update();
        }));
    }
}
