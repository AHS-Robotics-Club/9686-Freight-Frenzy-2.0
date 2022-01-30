package org.firstinspires.ftc.teamcode.auton.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestCarriage")
public class TestCarriage extends CommandOpMode {
    private SimpleServo leftDrop, rightDrop;

    @Override
    public void initialize() {
        leftDrop = new SimpleServo(hardwareMap, "leftDrop", -180, 180);
        rightDrop = new SimpleServo(hardwareMap, "rightDrop", -180, 180);

        schedule(new SequentialCommandGroup(new InstantCommand(() -> leftDrop.setPosition(0.5)).alongWith(new InstantCommand(() -> rightDrop.setPosition(0.5)), new InstantCommand(() -> leftDrop.setPosition(1)).alongWith(new InstantCommand(() -> rightDrop.setPosition(1))))));
    }
}
