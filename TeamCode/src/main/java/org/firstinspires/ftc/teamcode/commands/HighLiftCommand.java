package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

class LiftConstants {
    static double UP_TIME = 1;
    static double DOWN_TIME = 0.55;
}

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
            liftTime = LiftConstants.UP_TIME;
        } else if (lift.getLevel() == 2) {
            lift.motorDown();
            liftTime = LiftConstants.DOWN_TIME;
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
