package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

// import org.firstinspires.ftc.teamcode.custom.ServoGroup;
// import org.firstinspires.ftc.teamcode.SimpleServoEmpty;

public class DropSubsystem extends SubsystemBase {
    private SimpleServo leftDrop;
    private SimpleServo rightDrop;

    public static double dropOne = 130.0;


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


    public void dropTwo() { // 0.3333
//        leftDrop.turnToAngle(-10.0);
//        leftDrop.setPosition(0.333);
        // rightDrop.turnToAngle(-30.0);
//        leftDrop.setPosition(0.9);
        leftDrop.turnToAngle(-25.0);
    }

    public void dropThree() {
//        leftDrop.turnToAngle(-55.0);
//        leftDrop.setPosition(0.1);
        // rightDrop.turnToAngle(-85.0);
//        leftDrop.setPosition(0.7);
        leftDrop.turnToAngle(-105.0);
    }

    public void dropFour() { //
//        leftDrop.turnToAngle(-10.0);
        // rightDrop.turnToAngle(-10.0);
//        leftDrop.setPosition(0.1);
        leftDrop.turnToAngle(10.0);
    }

    public void dropOne() { // 0.6667
//        leftDrop.turnToAngle(60.0);
        // rightDrop.turnToAngle(30.0);
        leftDrop.turnToAngle(dropOne);
    }

    public void resetDrop() {
        leftDrop.turnToAngle(0.0);
        rightDrop.turnToAngle(0.0);
    }

    // 0.6667, 0.3333, 0.02777, -0.4444-, 0.6666

    public void initDropPos() {
        leftDrop.setPosition(0.6667);
    }

    public void midDropPos() {
        leftDrop.setPosition(0.3333);
    }

    public void dropPos() {
        leftDrop.setPosition(0.027777);
    }
}