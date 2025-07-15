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
import org.firstinspires.ftc.teamcode.commands.ServoFlipForwardCommand;
import org.firstinspires.ftc.teamcode.commands.ServoFlipSpecCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotUpCommand;
import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoFlipSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;


import java.util.Collections;

@Autonomous(name = "AutoBlue")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();
        // Initialize subsystems and Road Runner drive

        Pose2d initialPose = new Pose2d(23.85, -62.5, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        DriveSubsystem driveSubsystem = new DriveSubsystem(drive);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        liftSubsystem.resetEncoders();

        IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem(hardwareMap);
        ServoPivotSubsystem pivotSubsystem = new ServoPivotSubsystem(hardwareMap);
        ServoClawSubsystem servoClawSubsystem = new ServoClawSubsystem(hardwareMap);

        LinkageSubsystem linkageSubsystem = new LinkageSubsystem(hardwareMap);
        ServoFlipSubsystem servoFlipSubsystem = new ServoFlipSubsystem(hardwareMap);

/*
        LiftCommand liftDownCommand = new LiftCommand(liftSubsystem, 0); // Move lift down to 0 ticks
        LiftCommand liftBelowSpecimenCommand = new LiftCommand(liftSubsystem, 1300);
        LiftCommand liftSpecimenCommand = new LiftCommand(liftSubsystem, 2300);
        LiftCommand liftUpCommand = new LiftCommand(liftSubsystem, 3650);*/


        // Define trajectories as actions
/*


        ServoPivotLeftCommand servoPivotLeftCommand = new ServoPivotLeftCommand(pivotSubsystem);
        ServoPivotRightCommand servoPivotRightCommand = new ServoPivotRightCommand(pivotSubsystem);

        OpenClawCommand openServoCommand = new OpenClawCommand(servoClawSubsystem);
        CloseClawCommand closeServoCommand = new CloseClawCommand(servoClawSubsystem);
*/


        // start pivot right
        // lift below specimen
        TrajectoryActionBuilder testtrajectory = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(2, -30))
                .strafeTo(new Vector2d(35, -62.5))
                .strafeTo(new Vector2d(2, -28))
                .strafeTo(new Vector2d(35, -62.5))
                .strafeTo(new Vector2d(2, -28));

        TrajectoryActionBuilder preloadTraj = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-6, -28)); // go to bar


        // lift specimen
        // open claw
        // flip left
        // lift down

        TrajectoryActionBuilder pushSamplesTraj = preloadTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(-6, -36)) // back up
                .strafeTo(new Vector2d(36, -36)) //
                .strafeTo(new Vector2d(36, -14))
                .strafeTo(new Vector2d(47, -14))
                .strafeTo(new Vector2d(47, -52))
                .strafeTo(new Vector2d(47, -14))
                .strafeTo(new Vector2d(56, -14))
                .strafeTo(new Vector2d(56, -52))
                .strafeTo(new Vector2d(40, -52))
                .strafeTo(new Vector2d(40, -62.5));

        TrajectoryActionBuilder scoreSecondSpec = pushSamplesTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -28));

        TrajectoryActionBuilder returnAfterSecondSpec = scoreSecondSpec.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -52))
                .strafeTo(new Vector2d(40, -62.5));

        TrajectoryActionBuilder scoreThirdSpec = returnAfterSecondSpec.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -28));

        //TrajectoryActionBuilder score
        // cycle
        // grab left
        // flip right
        // lift below specimen



        // lift specimen
        // open claw
        // flip left



        // Wrap actions in commands
        ActionCommand preload = new ActionCommand(preloadTraj.build(), Collections.emptySet());
        ActionCommand pushSamples = new ActionCommand(pushSamplesTraj.build(), Collections.emptySet());
        ActionCommand scoreSecond = new ActionCommand(scoreSecondSpec.build(), Collections.emptySet());
        ActionCommand returnAfterSecond = new ActionCommand(returnAfterSecondSpec.build(), Collections.emptySet());
        ActionCommand scoreThird = new ActionCommand(scoreThirdSpec.build(), Collections.emptySet());

        // Combine commands into a sequence
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new CloseClawCommand(servoClawSubsystem),
                new WaitCommand(300),
            new ParallelCommandGroup(
                    new ServoPivotUpCommand(pivotSubsystem),
                    new ServoFlipForwardCommand(servoFlipSubsystem),
                    new LiftCommand(liftSubsystem, 500),
                    preload
            ),
            new SequentialCommandGroup(
                    new LiftCommand(liftSubsystem, 1150),
                    new WaitCommand(200),
                    new ParallelCommandGroup(
                            new OpenClawCommand(servoClawSubsystem),
                            new ServoFlipSpecCommand(servoFlipSubsystem),
                            new LiftCommand(liftSubsystem, 0),
                            new ServoPivotDownCommand(pivotSubsystem)
                    )
            ),
            pushSamples,
                new CloseClawCommand(servoClawSubsystem),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new ServoPivotUpCommand(pivotSubsystem),
                        new ServoFlipForwardCommand(servoFlipSubsystem),
                        new LiftCommand(liftSubsystem, 500),
                        scoreSecond
                ),
            new SequentialCommandGroup(
                    new LiftCommand(liftSubsystem, 1150),
                    new WaitCommand(200),
                    new ParallelCommandGroup(
                            new OpenClawCommand(servoClawSubsystem),
                            new ServoFlipSpecCommand(servoFlipSubsystem),
                            new LiftCommand(liftSubsystem, 0),
                            new ServoPivotDownCommand(pivotSubsystem)
                    )
            ),
            returnAfterSecond,
                new CloseClawCommand(servoClawSubsystem),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new ServoPivotUpCommand(pivotSubsystem),
                        new ServoFlipForwardCommand(servoFlipSubsystem),
                        new LiftCommand(liftSubsystem, 500),
                        scoreThird
                ),
            new SequentialCommandGroup(
                    new LiftCommand(liftSubsystem, 1150),
                    new WaitCommand(200),
                    new ParallelCommandGroup(
                            new OpenClawCommand(servoClawSubsystem),
                            new ServoFlipSpecCommand(servoFlipSubsystem),
                            new LiftCommand(liftSubsystem, 0),
                            new ServoPivotDownCommand(pivotSubsystem)
                    )
            )





//                    new CloseClawCommand(servoClawSubsystem),
//                    new ServoPivotUpCommand(pivotSubsystem),
//                    new LiftCommand(liftSubsystem, 1300),
//                        moveToBarCommand,
//                        //
//                        new LiftCommand(liftSubsystem, 2300),
//                        new OpenClawCommand(servoClawSubsystem),
//                        new WaitCommand(500),
//                        new ServoPivotDownCommand(pivotSubsystem),
//                        new LiftCommand(liftSubsystem, 0),
//                        pushToObs,
//                        new CloseClawCommand(servoClawSubsystem)

        );



        if (opModeIsActive()) {
            CommandScheduler.getInstance().schedule(autoSequence);
            while (opModeIsActive() && !autoSequence.isFinished()) {
                CommandScheduler.getInstance().run();
            }
        }
    }
}
