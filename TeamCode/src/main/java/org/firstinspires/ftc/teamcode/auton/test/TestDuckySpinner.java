package org.firstinspires.ftc.teamcode.auton.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;

@Config // Add this to make config values
@Autonomous(name = "TestDuckySpinner")
public class TestDuckySpinner extends CommandOpMode {
    private DuckySpinnerSubsystem duckySpinnerSubsystem;
    public static double POWER = 0.45; // Making variable public allows it to be changed in FTC Dashboard itself

    @Override
    public void initialize() {
        duckySpinnerSubsystem = new DuckySpinnerSubsystem(new Motor(hardwareMap, "duckySpinner"));

        schedule(new WaitUntilCommand(this::isStarted).andThen(new InstantCommand(() -> duckySpinnerSubsystem.run(POWER)).alongWith(new WaitCommand(2950)).andThen(new InstantCommand(() -> duckySpinnerSubsystem.stop()))));
        schedule(
                new WaitUntilCommand(this::isStarted)
                .andThen((new InstantCommand(() -> duckySpinnerSubsystem.run(POWER))
                .alongWith(new WaitCommand(2950))
                .andThen(new InstantCommand(() -> duckySpinnerSubsystem.stop()))))
        );
    }
}
