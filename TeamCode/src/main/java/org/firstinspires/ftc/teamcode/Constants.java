package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

public final class Constants {

    public enum HubLevel {
        LOW,
        MID,
        HIGH
    }

    public enum CapstoneLocation {
        LEFT,
        MID,
        RIGHT,
        NOT_FOUND,
    }

    // Time is in milliseconds
    public static class LiftConstants {
        public static final long MID_GOAL_UP = 700;
        public static final long MID_GOAL_DOWN = 350;
        public static final long HIGH_GOAL_UP = 1000;
        public static final long HIGH_GOAL_DOWN = 550;

        // liftMidUp = new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
        // liftMidDown = new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.MID_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop()))),
        // liftHighUp = new InstantCommand(() -> lift.motorUp()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_UP).andThen(new InstantCommand(() -> lift.motorStop()))),
        // liftHighDown = new InstantCommand(() -> lift.motorDown()).andThen(new WaitCommand((long) Constants.LiftConstants.HIGH_GOAL_DOWN).andThen(new InstantCommand(() -> lift.motorStop())))
    }
}
