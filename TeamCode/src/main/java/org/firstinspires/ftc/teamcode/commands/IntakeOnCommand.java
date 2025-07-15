package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class IntakeOnCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeOnCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.on();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

