package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private LiftSubsystem subsystem;
    private int level;
    private Telemetry telemetry;

    public LiftCommand(LiftSubsystem subsystem, Telemetry telemetry) {
        this.subsystem = subsystem;
        level = 0;
        this.telemetry = telemetry;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (level == 0) {
            subsystem.setGoal(0.5);
            level++;
            telemetry.addData("Help", level);
        } else if (level == 1) {
            subsystem.setGoal(1);
            level++;
            telemetry.addData("Help", level);

        } else if (level == 2) {
            subsystem.setGoal(10);
            level = 0;
            telemetry.addData("Help", level);

        }
    }
}