package org.firstinspires.ftc.teamcode.auton.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.HighLiftCommand;
import org.firstinspires.ftc.teamcode.commands.SecureBlockCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

@Autonomous(name = "TestLift")
public class TestLift extends CommandOpMode {

    @Override
    public void initialize() {

        ElapsedTime time = new ElapsedTime();
        time.reset();

        LiftSubsystemNoPID lift = new LiftSubsystemNoPID(
                new Motor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_435)
        );

        DropSubsystem drop = new DropSubsystem(
                new SimpleServo(hardwareMap, "leftDrop", -180, 180),
                new SimpleServo(hardwareMap, "rightDrop", -180, 180)
        );

        schedule(
                new SecureBlockCommand(lift, drop, time).raceWith(new WaitCommand(1000)).andThen(new WaitCommand(1000)),
                new SecureBlockCommand(lift, drop, time).raceWith(new WaitCommand(1000)).andThen(new WaitCommand(1000)),
                // new SecureBlockCommand(lift, drop, time).andThen(new WaitCommand(1000)),
                // new SecureBlockCommand(lift, drop, time).andThen(new WaitCommand(1000)),
                new SecureBlockCommand(lift, drop, time),
                new HighLiftCommand(lift, drop, time),
                new InstantCommand(drop::dropThree).alongWith(new WaitCommand(1000)),
                new InstantCommand(drop::dropTwo).alongWith(new WaitCommand(1000)),
                new HighLiftCommand(lift, drop, time),
                new SecureBlockCommand(lift, drop, time).andThen(new WaitCommand(1000))
        );

        schedule(new RunCommand(() -> telemetry.addData("Lift Level", lift.getLevel())));

    }

}
