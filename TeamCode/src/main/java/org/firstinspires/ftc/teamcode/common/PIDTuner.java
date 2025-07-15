package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDTuner extends OpMode {
    private PIDController controller;

    public static double p = 0.009, i = 0, d = 0.0001;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop() {
        controller.setPID(p, i , d);
        int liftPos = liftMotor2.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degree)) * f;

        double power = pid + ff;

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        telemetry.addData("pos", liftPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}
