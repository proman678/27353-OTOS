package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final int targetPosition;
    private static final int TOLERANCE = 10;

    public LiftCommand(LiftSubsystem liftSubsystem, int targetPosition) {
        this.liftSubsystem = liftSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() {
        liftSubsystem.update();
    }

    @Override
    public boolean isFinished() {
        int currentPosition = liftSubsystem.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= TOLERANCE;
        //return true;
    }

    @Override
    public void end(boolean interrupted) {
            liftSubsystem.stop();
    }
}
