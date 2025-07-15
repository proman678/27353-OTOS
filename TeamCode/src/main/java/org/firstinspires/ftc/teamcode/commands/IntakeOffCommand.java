package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class IntakeOffCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeOffCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.off();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

