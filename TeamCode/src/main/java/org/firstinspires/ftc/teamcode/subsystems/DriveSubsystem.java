package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

// Every subsystem is an object
public class DriveSubsystem extends SubsystemBase {

    // Declare each variable going to be used
    private MecanumDrive mecanumDrive;
    private Motor frontLeft, frontRight, backLeft, backRight;
    private RevIMU revIMU;

    // Constructor takes in the hardware that makes up the subsystem
    public DriveSubsystem(Motor fL, Motor fR, Motor bL, Motor bR, RevIMU imu) {
        // Set parameters equal to declared variables for use in methods
        frontLeft = fL;
        frontRight = fR;
        backLeft = bL;
        backRight = bR;

        revIMU = imu;

        // Initialize mecanum drive object
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    // Method which runs the subsystem
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mecanumDrive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, true);
    }



}
