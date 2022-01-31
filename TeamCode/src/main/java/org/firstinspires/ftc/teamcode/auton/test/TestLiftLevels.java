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

import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

// Time is in milliseconds
class LevelConstants {
//    static long MID_GOAL_UP = 500;
//    static long MID_GOAL_DOWN = 550;
//    static long HIGH_GOAL_UP = 800;
//    static long HIGH_GOAL_DOWN = 850;
    static long MID_GOAL_UP = 1000;
    static long MID_GOAL_DOWN = 550;
    static long HIGH_GOAL_UP = 1600;
    static long HIGH_GOAL_DOWN = 750;
}

@Autonomous(name = "Lift Test")
public class TestLiftLevels extends CommandOpMode {

    private LiftSubsystemNoPID lift;
    private DropSubsystem drop;
    private ElapsedTime time;

    @Override
    public void initialize() {
        lift = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435)
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
                new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand(LevelConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropThree()).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand(LevelConstants.MID_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropTwo()).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand(LevelConstants.HIGH_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropThree()).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand(LevelConstants.HIGH_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))),
                new InstantCommand(() -> drop.dropFour()).andThen(new WaitCommand(2500))
        )));
    }
}
