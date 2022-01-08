package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;

public class DropCommand extends CommandBase {
    private DropSubsystem dropS;
    private int dds = 0;
    private static int dropCount = 0;

    public DropCommand(DropSubsystem dropSubsystem) {
        dropS = dropSubsystem;
        addRequirements(dropS);
    }

    @Override
    public void execute() {
        dropCount = dds;

        if (dropCount == 0) {
            dropS.miniDrop();
            dropCount++;
        } else if (dropCount == 1) {
            dropS.initDrop();
            dropCount++;
        } else if (dropCount == 2) {
            dropS.halfDrop();
            dropCount++;
        } else {
            dropS.drop();
            dropCount = 0;
        }
    }

    public void end(boolean interrupted) {
        dds = dropCount;
    }

}


