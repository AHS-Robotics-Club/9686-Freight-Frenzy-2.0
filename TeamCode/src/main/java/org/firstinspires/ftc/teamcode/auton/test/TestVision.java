package org.firstinspires.ftc.teamcode.auton.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auton.e_vision.FFCapstoneDetector;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "TestVision")
public class TestVision extends CommandOpMode {

    private FFCapstoneDetector capstoneDetector;

    @Override
    public void initialize() {

        capstoneDetector = new FFCapstoneDetector(hardwareMap, "bigBrother");
        capstoneDetector.init();
        capstoneDetector.setLowerAndUpperBounds(new Scalar(20, 142, 20), new Scalar(255, 163, 90));

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.addData("Lift Pos", capstoneDetector.placementId());
            telemetry.update();
        })));
    }
}