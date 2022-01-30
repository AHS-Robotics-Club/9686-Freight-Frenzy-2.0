package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.e_vision.FFCapstoneDetector;
import org.firstinspires.ftc.teamcode.auton.paths.BlueCarouselPath;
import org.firstinspires.ftc.teamcode.auton.paths.RedCarouselPath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.opencv.core.Scalar;

import java.util.HashMap;

@Autonomous(name="BlueCarouselAutonVision")
public class BlueCarouselAutonVision extends CommandOpMode{
    private Motor mDuckySpinner;
    private Motor mIntake;
    private LiftSubsystemNoPID liftSubsystem;
    private Motor mRightLift, mLeftLift;

    private SimpleServo sLeftDrop, sRightDrop;

    private MecanumDriveSubsystem drive;
    private DuckySpinnerSubsystem duckySpinnerSubsystem;
    private DropSubsystem dropSubsystem;

    private FFCapstoneDetector capstoneDetector;
    private FFCapstoneDetector.Placement location;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initialize() {
        mDuckySpinner = new Motor(hardwareMap, "duckySpinner");
        mIntake = new Motor(hardwareMap, "intake");

        liftSubsystem = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435)
        );

        sLeftDrop = new SimpleServo(hardwareMap, "leftDrop", -180, 180);
        sRightDrop = new SimpleServo(hardwareMap, "rightDrop", -180, 180);

        dropSubsystem = new DropSubsystem(sLeftDrop, sRightDrop);
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        duckySpinnerSubsystem = new DuckySpinnerSubsystem(mDuckySpinner);

        capstoneDetector = new FFCapstoneDetector(hardwareMap, "bigBrother");
        capstoneDetector.init();
        capstoneDetector.setLowerAndUpperBounds(new Scalar(20, 142, 20), new Scalar(255, 163, 90));

        // FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.addData("Lift Pos", capstoneDetector.placementId());
            telemetry.update();
        })));

//        SequentialCommandGroup auton = new RedCarouselPath(drive, dropSubsystem, mIntake, mDuckySpinner);
//
//
//        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500)).andThen(auton));

//        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
//            telemetry.addData("location", location);
//            telemetry.update();
//        })));

//        schedule(auton.andThen(auton1));
        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500))
                .andThen(new RunCommand(() -> location = capstoneDetector.getPlacement())).raceWith(new WaitCommand(500))
//                .andThen(new InstantCommand(() -> {
//                    dashboardTelemetry.addData("Location", location);
//                    dashboardTelemetry.update();
//                }))
                .andThen(new SelectCommand(new HashMap<Object, Command>(){{
                    put(FFCapstoneDetector.Placement.LEFT,
                        new BlueCarouselPath(drive, dropSubsystem, mIntake, mDuckySpinner, liftSubsystem, 2));
                    put(FFCapstoneDetector.Placement.CENTER,
                        new BlueCarouselPath(drive, dropSubsystem, mIntake, mDuckySpinner, liftSubsystem, 1));
                    put(FFCapstoneDetector.Placement.RIGHT,
                        new BlueCarouselPath(drive, dropSubsystem, mIntake, mDuckySpinner, liftSubsystem, 0));
                }}, () -> capstoneDetector.getPlacement()))

        );
    }
}

