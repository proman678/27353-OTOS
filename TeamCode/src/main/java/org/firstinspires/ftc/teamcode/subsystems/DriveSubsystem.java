package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;

public class DriveSubsystem extends SubsystemBase {
    private final SparkFunOTOSDrive drive;

    public DriveSubsystem(SparkFunOTOSDrive drive) {
        this.drive = drive;
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }
}
