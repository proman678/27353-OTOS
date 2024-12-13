package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotLeftCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotRightCommand;
import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;


import java.util.Collections;

@Autonomous(name = "AutoBlue")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize subsystems and Road Runner drive
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        DriveSubsystem driveSubsystem = new DriveSubsystem(drive);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        liftSubsystem.resetEncoders();

        LiftCommand liftDownCommand = new LiftCommand(liftSubsystem, 0); // Move lift down to 0 ticks
        LiftCommand liftBelowSpecimenCommand = new LiftCommand(liftSubsystem, 1300);
        LiftCommand liftSpecimenCommand = new LiftCommand(liftSubsystem, 2300);
        LiftCommand liftUpCommand = new LiftCommand(liftSubsystem, 3650);


        // Define trajectories as actions
        ServoPivotSubsystem pivotSubsystem = new ServoPivotSubsystem(hardwareMap);
        ServoClawSubsystem servoClawSubsystem = new ServoClawSubsystem(hardwareMap);

        ServoPivotLeftCommand servoPivotLeftCommand = new ServoPivotLeftCommand(pivotSubsystem);
        ServoPivotRightCommand servoPivotRightCommand = new ServoPivotRightCommand(pivotSubsystem);

        OpenClawCommand openServoCommand = new OpenClawCommand(servoClawSubsystem);
        CloseClawCommand closeServoCommand = new CloseClawCommand(servoClawSubsystem);


        // start pivot right
        // lift below specimen


        TrajectoryActionBuilder trajectory1 = driveSubsystem.actionBuilder(new Pose2d(0, -61.5, Math.toRadians(180)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -34)) // go to bar
                .waitSeconds(1);

        // lift specimen
        // open claw
        // flip left
        // lift down

        TrajectoryActionBuilder trajectory2 = trajectory1.endTrajectory().fresh()
                .strafeTo(new Vector2d(38, -34))
                .strafeTo(new Vector2d(38, -11))
                .strafeTo(new Vector2d(47, -11))
                .strafeTo(new Vector2d(47, -52))
                .strafeTo(new Vector2d(47, -11))
                .strafeTo(new Vector2d(56, -11))
                .strafeTo(new Vector2d(56, -52))
                .strafeTo(new Vector2d(56, -11))
                .strafeTo(new Vector2d(62, -11))
                .strafeTo(new Vector2d(62, -62));

        // cycle
        // grab left
        // flip right
        // lift below specimen

        TrajectoryActionBuilder trajectory3 =  trajectory2.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -34));

        // lift specimen
        // open claw
        // flip left
        // lift down

        TrajectoryActionBuilder trajectory4 =  trajectory3.endTrajectory().fresh()
                .strafeTo(new Vector2d(38, -62));

        // grab left
        // lift below specimen
        // flip right

        TrajectoryActionBuilder trajectory5 =  trajectory4.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -34));

        // lift specimen
        // open claw
        // flip left



        // Wrap actions in commands
        ActionCommand moveToBarCommand = new ActionCommand(trajectory1.build(), Collections.emptySet());
        ActionCommand moveToHPCommand = new ActionCommand(trajectory2.build(), Collections.emptySet());

        // Combine commands into a sequence
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        closeServoCommand,
                        servoPivotRightCommand,
                        moveToBarCommand,
                        liftBelowSpecimenCommand
                ),
                new SequentialCommandGroup(
                        liftSpecimenCommand,
                        moveToHPCommand
                )
        );

        waitForStart();

        if (opModeIsActive()) {
            CommandScheduler.getInstance().schedule(autoSequence);
            while (opModeIsActive() && !autoSequence.isFinished()) {
                CommandScheduler.getInstance().run();
            }
        }
    }
}
