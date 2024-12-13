package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotLeftCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotRightCommand;
import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftState;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

@TeleOp(name = "Command-Based Lift TeleOp", group = "TeleOp")
public class TeleOpCommandBased extends OpMode {

    private DriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;

    private ServoPivotSubsystem pivotSubsystem;
    private LiftCommand liftUpCommand;
    private LiftCommand liftDownCommand;

    private LiftCommand liftBelowSpecimenCommand;
    private LiftCommand liftSpecimenCommand;

    private ServoClawSubsystem servoClawSubsystem;
    private SparkFunOTOSDrive sparkFunDrive;

    private ServoPivotLeftCommand servoPivotLeftCommand;
    private ServoPivotRightCommand servoPivotRightCommand;

    private OpenClawCommand openServoCommand;

    private CloseClawCommand closeServoCommand;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private LiftState currentLiftState = LiftState.DOWN;
    @Override
    public void start() {
        CommandScheduler.getInstance().reset();

        // Initialize the lift subsystem
        sparkFunDrive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        driveSubsystem = new DriveSubsystem(sparkFunDrive);
        liftSubsystem = new LiftSubsystem(hardwareMap);
        liftSubsystem.resetEncoders();

        pivotSubsystem = new ServoPivotSubsystem(hardwareMap);
        servoClawSubsystem = new ServoClawSubsystem(hardwareMap);

        // Define the lift commands with appropriate target positions
         // Move lift up to -600 ticks
/*
        liftDownCommand = new LiftCommand(liftSubsystem, 0); // Move lift down to 0 ticks
        liftBelowSpecimenCommand = new LiftCommand(liftSubsystem, 1300);
        liftSpecimenCommand = new LiftCommand(liftSubsystem, 2300);
        liftUpCommand = new LiftCommand(liftSubsystem, 3650);
*/



        // game pads
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new LiftCommand(liftSubsystem, 0).alongWith(
                new InstantCommand(() -> currentLiftState = LiftState.DOWN)
        ));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new LiftCommand(liftSubsystem, 1300).alongWith(
                new InstantCommand(() -> currentLiftState = LiftState.BELOW_SPECIMEN)
        ));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new LiftCommand(liftSubsystem, 2300).alongWith(
                new InstantCommand(() -> currentLiftState = LiftState.SPECIMEN)
        ));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LiftCommand(liftSubsystem, 3650).alongWith(
                new InstantCommand(() -> currentLiftState = LiftState.UP)
        ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ServoPivotLeftCommand(pivotSubsystem));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ServoPivotRightCommand(pivotSubsystem));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new OpenClawCommand(servoClawSubsystem),
                        new WaitCommand(500),
                        new ServoPivotLeftCommand(pivotSubsystem),
                        new LiftCommand(liftSubsystem, 0).alongWith(
                                new InstantCommand(() -> currentLiftState = LiftState.DOWN)
                        )
                )
        );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new CloseClawCommand(servoClawSubsystem),
                        new WaitCommand(500),
                        new ServoPivotRightCommand(pivotSubsystem),
                        new LiftCommand(liftSubsystem, 1300).alongWith(
                                new InstantCommand(() -> currentLiftState = LiftState.BELOW_SPECIMEN)
                        )
                )
        );

    }

    @Override
    public void init() {

        telemetry.addData("Status", "TeleOp Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Use the CommandScheduler to handle commands
        CommandScheduler.getInstance().run();

        sparkFunDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        sparkFunDrive.updatePoseEstimate();
        // Display telemetry
        //telemetry.addData("Lift Position", liftSubsystem.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "TeleOp Stopped");
        telemetry.update();
    }
}
