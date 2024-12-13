package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx liftMotor1;
    private final DcMotorEx liftMotor2; // Second motor for the lift
    private final PIDFController pidController;

    // PID coefficients
    private static final double kP = 0.005;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kF = 0.0;

    // Desired position (encoder ticks)
    private int setpoint = 0;


    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        // Reset and configure motors
        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse one motor to ensure they move the lift shaft in the same direction
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the PIDFController
        pidController = new PIDFController(kP, kI, kD, kF);
    }
    public void setTargetPosition(int targetPosition) {
        this.setpoint = targetPosition;
        pidController.setSetPoint(setpoint);
    }

    public void stop() {
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        // Get current position and calculate the PID output
        int currentPosition = liftMotor1.getCurrentPosition(); // Assume both motors have synchronized encoders
        double power = pidController.calculate(currentPosition, setpoint);

        // Set power to both motors
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public boolean atSetpoint() {
        return pidController.atSetPoint();
    }
}
