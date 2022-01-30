package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;

public class DropCommand extends CommandBase {
    private DropSubsystem dropS;
    private int dropCount = 0;

    public DropCommand(DropSubsystem dropSubsystem) {
        dropS = dropSubsystem;
        addRequirements(dropS);
    }

    @Override
    public void initialize() {
        if (dropCount == 0)
            dropS.dropTwo();
        else if (dropCount == 1)
            dropS.dropThree();
        else
            dropS.dropOne();
    }

    public void end(boolean interrupted) {
        if (dropCount != 2)
            dropCount++;
        else
            dropCount = 0;
    }

}


