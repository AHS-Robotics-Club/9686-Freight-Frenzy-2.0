package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auton.e_vision.FFCapstoneDetector;
import org.firstinspires.ftc.teamcode.auton.paths.BlueIntakePath;
import org.firstinspires.ftc.teamcode.auton.paths.RedCarouselPath;
import org.firstinspires.ftc.teamcode.auton.paths.RedIntakePath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.opencv.core.Scalar;

import java.util.HashMap;

@Autonomous(name = "RedIntakeAutoVision")
public class RedIntakeAutoVision extends CommandOpMode {

    private SimpleServo sDropLeft, sDropRight;
    private Motor mIntake;

    private MecanumDriveSubsystem drive;
    private DropSubsystem dropSubsystem;
    private LiftSubsystemNoPID liftSubsystem;

    private FFCapstoneDetector capstoneDetector;
    private FFCapstoneDetector.Placement location;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        sDropLeft = new SimpleServo(hardwareMap, "leftDrop", -90, 90);
        sDropRight = new SimpleServo(hardwareMap, "rightDrop", -90, 90);
        mIntake = new Motor(hardwareMap, "intake");

        dropSubsystem = new DropSubsystem(sDropLeft, sDropRight);
        liftSubsystem = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "leftLift"),
                new Motor(hardwareMap, "rightLift")
        );

        capstoneDetector = new FFCapstoneDetector(hardwareMap, "bigBrother");
        capstoneDetector.init();
        capstoneDetector.setLowerAndUpperBounds(new Scalar(20, 142, 20), new Scalar(255, 163, 90));

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("location", location);
            telemetry.update();
        })));

//        schedule(auton.andThen(auton1));
        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500))
                .andThen(new RunCommand(() -> location = capstoneDetector.getPlacement())).raceWith(new WaitCommand(500))
                .andThen(new SelectCommand(new HashMap<Object, Command>(){{
                    put(FFCapstoneDetector.Placement.LEFT,
                            new RedIntakePath(drive, dropSubsystem, mIntake, liftSubsystem, 0));
                    put(FFCapstoneDetector.Placement.CENTER,
                            new RedIntakePath(drive, dropSubsystem, mIntake, liftSubsystem, 1));
                    put(FFCapstoneDetector.Placement.RIGHT,
                            new RedIntakePath(drive, dropSubsystem, mIntake, liftSubsystem, 2));
                }}, () -> capstoneDetector.getPlacement()))
        );
    }
}
