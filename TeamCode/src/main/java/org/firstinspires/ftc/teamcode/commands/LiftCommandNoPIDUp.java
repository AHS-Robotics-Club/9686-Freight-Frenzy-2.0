package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class LiftCommandNoPIDUp extends CommandBase {

    private LiftSubsystemNoPID sLift;
    private ElapsedTime time;
    private double timeToLift;

    public LiftCommandNoPIDUp(LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime time) {
        sLift = liftSubsystemNoPID;
        this.time = time;

        addRequirements(sLift);
    }

    @Override
    public void initialize() {
        time.reset();
        if(sLift.getLevel() == 0) {
            sLift.motorUp();
            timeToLift = 0.25;
        } else if (sLift.getLevel() == 1) {
            sLift.motorUp();
            timeToLift = 0.3;
        } else if (sLift.getLevel() == 2) {
            sLift.motorUp();
            timeToLift = 0.4;
        }
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= timeToLift;
    }

    @Override
    public void end(boolean interrupted) {
        if(sLift.getLevel() != 3) {
            sLift.motorStop();
            sLift.addLevel();
        }
    }
}