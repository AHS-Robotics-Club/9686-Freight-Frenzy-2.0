package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

// import org.firstinspires.ftc.teamcode.custom.ServoGroup;
// import org.firstinspires.ftc.teamcode.SimpleServoEmpty;

public class DropSubsystem extends SubsystemBase {
    private SimpleServo leftDrop;
    private SimpleServo rightDrop;


// private SimpleServoEmpty leftDropEmpty;
// private SimpleServoEmpty rightDropEmpty;
// private ServoGroup dropGroup;

    public DropSubsystem(SimpleServo leftDropServo, SimpleServo rightDropServo) {
        leftDrop = leftDropServo;
        rightDrop = rightDropServo;

        // TODO: Lift servos higher on init
        // rightDrop.turnToAngle(10.0);
    }

    // public DropSubsystem(SimpleServoEmpty leftDropServo, SimpleServoEmpty rightDropServo) {
    //     leftDropEmpty = leftDropServo;
    //     rightDropEmpty = rightDropServo;

    //     dropGroup = new ServoGroup(leftDropEmpty, rightDropEmpty);
    //     // TODO: Lift servos higher on init
    // }

    public void initDropLeft() {
        if (leftDrop != null)
            leftDrop.turnToAngle(10.0);
    }

    public void initDropRight() {
        if (rightDrop != null)
            rightDrop.turnToAngle(10.0);
    }

    // TODO: Test servo angles at start position.
    public void dropLeft() {
        if (leftDrop != null)
            leftDrop.turnToAngle(120.0);
    }

    public void dropRight() {
        if (rightDrop != null)
            rightDrop.turnToAngle(120.0);
    }

    public void resetDropLeft() {
        if (leftDrop != null)
            leftDrop.turnToAngle(0.0);
    }

    public void resetDropRight() {
        if (rightDrop != null)
            rightDrop.turnToAngle(0.0);
    }

    public void initDrop() {
        leftDrop.turnToAngle(-85.0);
        rightDrop.turnToAngle(-85.0);
    }

    public void drop() {
        leftDrop.turnToAngle(30.0);
        rightDrop.turnToAngle(30.0);
    }

    public void miniDrop() {
        leftDrop.turnToAngle(-30.0);
        rightDrop.turnToAngle(-30.0);
    }
    public void halfDrop() {
        leftDrop.turnToAngle(-10.0);
        rightDrop.turnToAngle(-10.0);
    }
    public void resetDrop() {
        leftDrop.turnToAngle(0.0);
        rightDrop.turnToAngle(0.0);
    }
}