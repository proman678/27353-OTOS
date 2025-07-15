package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class ServoPivotDownCommand extends CommandBase {
    private final ServoPivotSubsystem pivotSubsystem;

    public ServoPivotDownCommand(ServoPivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.moveToPositionDown();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

