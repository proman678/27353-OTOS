package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class MotorTest extends OpMode {

    DcMotorEx motor;
    public static String motorName = "motor";
    public static double power = 0.5;

    @Override
    public void init() {
        motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && power <= 1) {
            power += .05;

        } else if (gamepad1.dpad_down && power >= 0) {
            power -= .05;

        }

        motor.setPower(power);
        telemetry.addData("Motor power: ", power);
        telemetry.update();
    }
}