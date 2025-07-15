package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.commands.LinkageOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ServoFlipForwardCommand;
import org.firstinspires.ftc.teamcode.commands.ServoFlipSpecCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPivotUpCommand;
import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftState;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoFlipSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoPivotSubsystem;

@TeleOp(name = "Command-Based Lift TeleOp", group = "TeleOp")
public class TeleOpCommandBased extends OpMode {

    private DriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;

    private ServoPivotSubsystem pivotSubsystem;
    private ServoFlipSubsystem flipSubsystem;

    private LinkageSubsystem linkageSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private LinkageInCommand linkageInCommand;
    private LinkageOutCommand linkageDownCommand;



    private ServoFlipSpecCommand servoFlipSpecCommand;
    private ServoFlipForwardCommand servoFlipForwardCommand;

    private LiftCommand liftUpCommand;
    private LiftCommand liftDownCommand;

    private LiftCommand liftBelowSpecimenCommand;
    private LiftCommand liftSpecimenCommand;

    private ServoClawSubsystem servoClawSubsystem;
    private IntakePivotSubsystem intakePivotSubsystem;

    private SparkFunOTOSDrive sparkFunDrive;

    private ServoPivotDownCommand servoPivotDownCommand;
    private ServoPivotUpCommand servoPivotUpCommand;

    private OpenClawCommand openServoCommand;

    private CloseClawCommand closeServoCommand;

    private DcMotorEx intakeExtensionMotor;

    private DcMotorEx intakeSpinnerMotor;
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

        linkageSubsystem = new LinkageSubsystem(hardwareMap);
        pivotSubsystem = new ServoPivotSubsystem(hardwareMap);
        servoClawSubsystem = new ServoClawSubsystem(hardwareMap);
        intakePivotSubsystem = new IntakePivotSubsystem(hardwareMap);
        flipSubsystem = new ServoFlipSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);


/*        intakeExtensionMotor = hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");
        intakeSpinnerMotor = hardwareMap.get(DcMotorEx.class, "intakeSpinnerMotor");*/
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

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new LiftCommand(liftSubsystem, 0).alongWith(
//                new InstantCommand(() -> currentLiftState = LiftState.DOWN)
//        ));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new LiftCommand(liftSubsystem, 600).alongWith(
//                new InstantCommand(() -> currentLiftState = LiftState.BELOW_SPECIMEN)
//        ));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new LiftCommand(liftSubsystem, 1100).alongWith(
//                new InstantCommand(() -> currentLiftState = LiftState.SPECIMEN)
//        ));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LiftCommand(liftSubsystem, 3650).alongWith(
//                new InstantCommand(() -> currentLiftState = LiftState.UP)
//        ));

        //forward
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new CloseClawCommand(servoClawSubsystem),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new ServoFlipForwardCommand(flipSubsystem),
                                new ServoPivotUpCommand(pivotSubsystem)
                        ),
                new LiftCommand(liftSubsystem, 600)
                )
        );

        //spec
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new OpenClawCommand(servoClawSubsystem),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new ServoFlipSpecCommand(flipSubsystem),
                                new LiftCommand(liftSubsystem, 0),
                                new ServoPivotDownCommand(pivotSubsystem)
                        )
                )

        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new LiftCommand(liftSubsystem, 1100)
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new LiftCommand(liftSubsystem, 600)
        );





        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new LinkageOutCommand(linkageSubsystem)
        );
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new LinkageInCommand(linkageSubsystem)
        );




/*
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(
                        () -> intakeSpinnerMotor.setPower(-1)
                )).whenReleased(
                new InstantCommand(
                        () -> intakeSpinnerMotor.setPower(0)
                )
        );
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(
                        () -> intakeSpinnerMotor.setPower(0.4)
                )).whenReleased(
                new InstantCommand(
                        () -> intakeSpinnerMotor.setPower(0)
                )
        );*/
//
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new IntakePivotDownCommand(intakePivotSubsystem)
//        );
//
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new IntakePivotUpCommand(intakePivotSubsystem)
//        );




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
                        -gamepad1.left_stick_y*0.8,
                        -gamepad1.left_stick_x*0.8
                ),
                -gamepad1.right_stick_x*0.8
        ));

        sparkFunDrive.updatePoseEstimate();
        //intakeExtensionMotor.setPower(-gamepad2.left_stick_y);
        // Display telemetry
        //telemetry.addData("Lift Position", liftSubsystem.getCurrentPosition());
        telemetry.update();

        if(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new IntakePivotDownCommand(intakePivotSubsystem),
                            new IntakeOnCommand(intakeSubsystem)
                    )
            );
        }
        else if (gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new IntakePivotDownCommand(intakePivotSubsystem),
                            new IntakeReverseCommand(intakeSubsystem)
                    )
            );
        }
        else{
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new IntakePivotUpCommand(intakePivotSubsystem),
                            new IntakeOffCommand(intakeSubsystem)
                    )
            );

        }

    }

    @Override
    public void stop() {
        telemetry.addData("Status", "TeleOp Stopped");
        telemetry.update();
    }
}
