package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class LiftSubsystemNoPID extends SubsystemBase {
    private final Motor leftLift, rightLift;
    private MotorGroupTemp lift;
    private int level;

    public LiftSubsystemNoPID(Motor mLeftLift, Motor mRightLift) {
        leftLift = mLeftLift;
        rightLift = mRightLift;
        level = 0;

        lift = new MotorGroupTemp(leftLift, rightLift);
    }

    // TODO: Adjust lift system motor power, just for testing.
    public void motorUp() {
        lift.set(0.5);
    }

    public void motorDown() {
        lift.set(-0.5);
    }

    public void slightMotorUp() { lift.set(0.2); }

    public void slightMotorDown() { lift.set(-0.2); }

    public void motorStop() {
        lift.set(0);
    }

    public int getLevel() { return level; }

    public void addLevel() { level++; }

    public void resetLevel() { level = 0; }

}
