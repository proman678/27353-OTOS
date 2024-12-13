package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;

public class CloseClawCommand extends CommandBase {
    private final ServoClawSubsystem clawSubsystem;

    public CloseClawCommand(ServoClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}
