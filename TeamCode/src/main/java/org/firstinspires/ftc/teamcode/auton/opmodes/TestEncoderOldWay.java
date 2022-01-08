package org.firstinspires.ftc.teamcode.auton.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TestDrivetrainEncoder")
public class TestEncoderOldWay extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    final double turns = 384.5 * 20;

    @Override
    public void runOpMode() throws InterruptedException
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder counts kept by motors.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition((int) turns);
        frontRight.setTargetPosition((int) turns);
        backLeft.setTargetPosition((int) turns);
        backRight.setTargetPosition((int) turns);

        // set motors to run to target encoder position and stop with brakes on.
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        frontLeft.setPower(1);

        while (opModeIsActive() && frontLeft.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", frontLeft.getCurrentPosition() + "  busy=" + frontLeft.isBusy());
            telemetry.update();
            idle();
        }

        frontLeft.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", frontLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }

        frontRight.setPower(1);

        while (opModeIsActive() && frontRight.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-right", frontRight.getCurrentPosition() + "  busy=" + frontRight.isBusy());
            telemetry.update();
            idle();
        }

        frontRight.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        backLeft.setPower(1);

        while (opModeIsActive() && backLeft.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-right", backLeft.getCurrentPosition() + "  busy=" + backLeft.isBusy());
            telemetry.update();
            idle();
        }

        backLeft.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", backLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }

        backRight.setPower(1);

        while (opModeIsActive() && backRight.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-right", backRight.getCurrentPosition() + "  busy=" + backRight.isBusy());
            telemetry.update();
            idle();
        }

        backRight.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", backRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

    }
}
