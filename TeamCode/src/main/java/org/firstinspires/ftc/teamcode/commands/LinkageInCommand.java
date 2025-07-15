package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

public class LinkageInCommand extends CommandBase {
    private final LinkageSubsystem linkageSubsystem;

    public LinkageInCommand(LinkageSubsystem linkageSubsystem) {
        this.linkageSubsystem = linkageSubsystem;
        addRequirements(linkageSubsystem);
    }

    @Override
    public void initialize() {
        linkageSubsystem.moveToPositionIn();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

