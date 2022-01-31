package org.firstinspires.ftc.teamcode.auton.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MotorTest")
public class TestMotors extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backLeft = new Motor(hardwareMap, "backLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backRight = new Motor(hardwareMap, "backRight");

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        schedule(new SequentialCommandGroup(new InstantCommand(() -> frontLeft.set(1.0))
                .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> frontLeft.stopMotor()))),
                new InstantCommand(() -> backLeft.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> backLeft.stopMotor()))),
                new InstantCommand(() -> frontRight.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> frontRight.stopMotor()))),
                new InstantCommand(() -> backRight.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> backRight.stopMotor())))));

    }
}
