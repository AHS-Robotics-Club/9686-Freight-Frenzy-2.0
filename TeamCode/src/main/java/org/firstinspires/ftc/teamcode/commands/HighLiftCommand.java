package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MainTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class HighLiftCommand extends CommandBase {

    private LiftSubsystemNoPID lift;
    private DropSubsystem drop;
    private ElapsedTime time;
    private double liftTime;

    public HighLiftCommand(LiftSubsystemNoPID lift, DropSubsystem drop, ElapsedTime time) {
        this.lift = lift;
        this.drop = drop;
        this.time = time;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        // if(lift.getLevel() == 1) {
        //     lift.motorUp();
        //     liftTime = 0.2;
        // } else if (lift.getLevel() == 2) {
        //     lift.motorDown();
        //     liftTime = 0.35;
        // }
        time.reset();
        if (!MainTeleOp.isHigh) {
            lift.motorUp();
            liftTime = (Constants.LiftConstants.HIGH_GOAL_UP) / 1000.0;
        } else {
            lift.motorDown();
            liftTime = 0.72; //0.55
        }
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motorStop();
        MainTeleOp.isHigh = !MainTeleOp.isHigh;
        if(MainTeleOp.isHigh) {
            drop.dropThree();
        } else {
            drop.dropOne();
            MainTeleOp.wasHigh = true;
            SecureBlockCommand.isSecured = false;
        }
    }

}
