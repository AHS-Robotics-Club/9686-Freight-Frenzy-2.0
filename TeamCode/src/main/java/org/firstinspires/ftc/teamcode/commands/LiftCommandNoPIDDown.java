package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class LiftCommandNoPIDDown extends CommandBase {
    private LiftSubsystemNoPID sLift;
    private ElapsedTime time;
    private double timeToDrop = 0.25;

    public LiftCommandNoPIDDown(LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime time) {
        sLift = liftSubsystemNoPID;
        this.time = time;

        addRequirements(sLift);
    }

    @Override
    public void initialize() {
        time.reset();
        if (sLift.getLevel() == 0) {

        } else if (sLift.getLevel() == 1) {
            sLift.motorDown();
            timeToDrop = 0.2;
        } else if (sLift.getLevel() == 2) {
            sLift.motorDown();
            timeToDrop = 0.45;
        } else if (sLift.getLevel() == 3) {
            sLift.motorDown();
            timeToDrop = 0.8;
        }
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= timeToDrop;
    }

    @Override
    public void end(boolean interrupted) {
        if(sLift.getLevel() != 0) {
            sLift.motorStop();
            sLift.resetLevel();
        }
    }

}