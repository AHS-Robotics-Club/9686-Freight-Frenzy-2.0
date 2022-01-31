package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class SecureBlockCommand extends SequentialCommandGroup {
    private LiftSubsystemNoPID lift;
    private DropSubsystem drop;

    private LiftCommandNoPIDUp cLiftUp;
    private LiftCommandNoPIDDown cLiftDown;

    private boolean isSecured;
    private double liftTime;

    public SecureBlockCommand(LiftSubsystemNoPID lift, DropSubsystem drop, ElapsedTime time) {
        this.lift = lift;
        this.drop = drop;
        cLiftUp = new LiftCommandNoPIDUp(lift, time);
        cLiftDown = new LiftCommandNoPIDDown(lift, time);

        isSecured = false;
        liftTime = 0.0;

        addCommands(
                new FunctionalCommand(
                        () -> addRequirements(lift),
                        () -> {
                            time.reset();
                            if(lift.getLevel() == 0) {
                                lift.motorUp();
                                liftTime = 0.25;
                            } else if (lift.getLevel() == 1) {
                                lift.motorDown();
                                liftTime = 0.2;
                            }
                        },
                        interrupted -> {
                            lift.motorStop();
                            if (lift.getLevel() == 0) {
                                lift.addLevel();
                                isSecured = true;
                            } else if (lift.getLevel() == 1) {
                                lift.resetLevel();
                                isSecured = false;
                            }
                        },
                        () -> time.seconds() >= liftTime,
                        lift
                ),
                lift.getLevel() == 0 ? new InstantCommand(drop::dropTwo) : new InstantCommand(drop::dropOne)
        );
    }





}
