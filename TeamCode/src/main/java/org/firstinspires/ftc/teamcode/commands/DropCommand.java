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
    public void execute() {
        if (dropCount == 0) {
            dropS.miniDrop();
        } else if (dropCount == 1) {
            dropS.initDrop();
        } else if (dropCount == 2) {
            dropS.halfDrop();
        } else {
            dropS.drop();
        }
    }

    public void end(boolean interrupted) {
        if (dropCount != 3)
            dropCount++;
        else
            dropCount = 0;
    }

}


