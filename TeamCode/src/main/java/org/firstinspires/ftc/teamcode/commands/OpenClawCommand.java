package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;

public class OpenClawCommand extends CommandBase {
    private final ServoClawSubsystem clawSubsystem;

    public OpenClawCommand(ServoClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.open();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}
