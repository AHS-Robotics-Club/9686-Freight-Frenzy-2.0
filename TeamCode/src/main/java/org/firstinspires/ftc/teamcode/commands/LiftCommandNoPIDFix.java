package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class LiftCommandNoPIDFix extends CommandBase {

        private LiftSubsystemNoPID lift;
        private SimpleServo sLeftDrop, sRightDrop;
        private ElapsedTime time;
        private boolean isUp;
        private int level = 0;
        private double timeToLift = 0.4;

        public LiftCommandNoPIDFix(LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime timer, boolean isUp) {
            lift = liftSubsystemNoPID;
            time = timer;
            this.isUp = isUp;

            // TODO: This might be an issue
            addRequirements(liftSubsystemNoPID);
        }

        @Override
        public void initialize() {
            time.reset();
            if(isUp) {
                lift.slightMotorUp();
            } else {
                lift.slightMotorDown();
            }
            timeToLift = 0.3;
        }

        @Override
        public boolean isFinished() {
            return time.seconds() >= timeToLift;
        }

        @Override
        public void end(boolean interrupted) {
            lift.motorStop();
        }
}
