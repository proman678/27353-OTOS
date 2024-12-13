package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final int targetPosition;

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
        return liftSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
            liftSubsystem.stop();
    }
}
