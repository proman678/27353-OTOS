package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class IntakePivotUpCommand extends CommandBase {
    private final IntakePivotSubsystem intakePivotSubsystem;

    public IntakePivotUpCommand(IntakePivotSubsystem pivotSubsystem) {
        this.intakePivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        intakePivotSubsystem.moveToPositionUp();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

