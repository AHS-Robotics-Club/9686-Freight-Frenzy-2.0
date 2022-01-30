package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class DropCommandInitLift extends CommandBase {

    private DropSubsystem dropS;
    private LiftSubsystemNoPID liftS;
    private int dropCount = 0;
    private double timeToLift = 0;
    private ElapsedTime time;

    public DropCommandInitLift(DropSubsystem dropSubsystem, LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime time) {
        dropS = dropSubsystem;
        liftS = liftSubsystemNoPID;
        this.time = time;
        addRequirements(dropS, liftS);
    }

    @Override
    public void initialize() {
        if(liftS.getLevel() == 0) {
            time.reset();
            liftS.motorUp();
            timeToLift = 0.25;
        }
    }

    @Override
    public void execute() {
        if (dropCount == 0) {
            dropS.dropTwo();
        } else if (dropCount == 1) {
            dropS.dropThree();
        } else if (dropCount == 2) {
            dropS.dropFour();
        } else {
            dropS.dropOne();
        }
    }

    @Override
    public boolean isFinished() {
        if(liftS.getLevel() == 0) {
            return time.seconds() >= timeToLift;
        } else {
            return true;
        }
    }

    public void end(boolean interrupted) {
        if (dropCount != 3)
            dropCount++;
        else
            dropCount = 0;
    }
}
