package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.subsystems.liftPID.ProfiledPIDSubsystem;

public class LiftSubsystem extends ProfiledPIDSubsystem {

    private MotorGroupTemp lift;
    private static double kS = 1.0, kG = 1.0, kV = 1.0, kA = 1.0;
    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(kS, kG, kV, kA);
    private static double kP = 1.0, kI = 0, kD = 0;
    private final double distancePerPulse = Math.PI * 0.05 / 383.6;

    public LiftSubsystem(Motor mLeftLift, Motor mRightLift) {
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(5.0, 3.0)), 0.0);
        lift = new MotorGroupTemp(mLeftLift, mRightLift);
        lift.encoder.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setPoint) {
        double feedForward = elevatorFeedForward.calculate(setPoint.velocity);
        lift.set(((output + feedForward) / distancePerPulse) / lift.ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    @Override
    protected double getMeasurement() {
        return lift.encoder.getDistance();
    }

    public void setMotor(){
        lift.set(0.5);
    }

}
