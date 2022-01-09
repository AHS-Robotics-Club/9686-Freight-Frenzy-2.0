package org.firstinspires.ftc.teamcode.auton.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "BlueParkingTimed")
public class BlueParkingTimed extends CommandOpMode {

    private Motor fL, fR, bL, bR;
    private DriveSubsystem drive;
    private RevIMU imu;
    private ElapsedTime time;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");
        imu = new RevIMU(hardwareMap);
        drive = new DriveSubsystem(fL, fR, bL, bR, imu);

        time = new ElapsedTime();

        time.reset();

        SequentialCommandGroup scg = new SequentialCommandGroup(
                new RunCommand(() -> drive.drive(0.25, 0.75, 0)).withTimeout(1250)
        );

        schedule(scg);
    }
}
