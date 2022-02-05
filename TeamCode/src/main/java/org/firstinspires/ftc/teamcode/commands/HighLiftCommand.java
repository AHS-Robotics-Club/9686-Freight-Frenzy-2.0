package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class HighLiftCommand extends CommandBase {

    private LiftSubsystemNoPID lift;
    private ElapsedTime time;
    private double liftTime;

    public HighLiftCommand(LiftSubsystemNoPID lift, ElapsedTime time) {
        this.lift = lift;
        this.time = time;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if(lift.getLevel() == 1) {
            lift.motorUp();
            liftTime = Constants.LiftConstants.HIGH_GOAL_UP;
        } else if (lift.getLevel() == 2) {
            lift.motorDown();
            liftTime = Constants.LiftConstants.HIGH_GOAL_DOWN;
        }
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motorStop();
        if (lift.getLevel() == 1) {
            lift.addLevel();
        } else if (lift.getLevel() == 2) {
            lift.resetLevel();
        }
    }

}
