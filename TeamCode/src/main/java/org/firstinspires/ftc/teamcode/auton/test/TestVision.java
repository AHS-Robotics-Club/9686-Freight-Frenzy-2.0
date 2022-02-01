package org.firstinspires.ftc.teamcode.auton.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auton.vision.FFCapstoneDetector;

@Autonomous(name = "TestVision")
public class TestVision extends CommandOpMode {

    private FFCapstoneDetector capstoneDetector;

    @Override
    public void initialize() {

        capstoneDetector = new FFCapstoneDetector(hardwareMap, "bigBrother");
        capstoneDetector.init();

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.addData("Lift Pos", capstoneDetector.placementId());
            telemetry.update();
        })));
    }
}