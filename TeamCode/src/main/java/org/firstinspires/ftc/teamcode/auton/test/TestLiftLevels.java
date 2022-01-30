package org.firstinspires.ftc.teamcode.auton.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

@Autonomous(name = "LiftTest")
public class TestLiftLevels extends CommandOpMode {

    private LiftSubsystemNoPID lift;
    private DropSubsystem drop;
    private ElapsedTime time;

    @Override
    public void initialize() {
        lift = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435)
        );
        drop = new DropSubsystem(
                new SimpleServo(hardwareMap, "leftDrop", -180, 180),
                new SimpleServo(hardwareMap, "rightDrop", -180, 180)
        );
        time = new ElapsedTime();

        // schedule(new WaitUntilCommand(this::isStarted).andThen(new SequentialCommandGroup(
        //         new InstantCommand(() -> time.reset())),
        //         new ConditionalCommand(new InstantCommand(() -> lift.motorUp()), new InstantCommand(() -> lift.motorStop()), () -> time.seconds() <= 0.55).andThen(
        //
        //         )
        // ));

        schedule(new WaitUntilCommand(this::isStarted).andThen(new SequentialCommandGroup(
                new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropThree()).alongWith(new WaitCommand(1000)),
                new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropTwo()).alongWith(new WaitCommand(1000)),
                new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropThree()).andThen(new WaitCommand(1000)),
                new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop())))
        )));
    }
}
