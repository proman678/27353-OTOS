package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ServoFlipSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class ServoFlipSpecCommand extends CommandBase {
    private final ServoFlipSubsystem flipSubsystem;

    public ServoFlipSpecCommand(ServoFlipSubsystem flipSubsystem) {
        this.flipSubsystem = flipSubsystem;
        addRequirements(flipSubsystem);
    }

    @Override
    public void initialize() {
        flipSubsystem.moveToPositionSpec();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

