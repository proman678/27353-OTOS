package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class ServoPivotLeftCommand extends CommandBase {
    private final ServoPivotSubsystem pivotSubsystem;

    public ServoPivotLeftCommand(ServoPivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.moveToPositionLeft();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

