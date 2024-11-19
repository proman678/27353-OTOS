/*
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Ver.1.3")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double PPR = 384.5;
        double pi = 3.14159;
        Motor FL = new Motor(hardwareMap, "FL", PPR, "left");
        Motor FR = new Motor(hardwareMap, "FR", PPR, "right");
        Motor BL = new Motor(hardwareMap, "BL", PPR, "left");
        Motor BR = new Motor(hardwareMap, "BR", PPR, "right");

        Motor slideMotor = new Motor(hardwareMap, "slideMotor", PPR, "right");
        slideMotor.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Servo continuousServo = hardwareMap.get(Servo.class, "intakeServo");

        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        // Replace "servo_name" with the configured name of your servo
        */
/*
        Servo rightLink = hardwareMap.get(Servo.class, "rightLink"); // Replace "servo_name" with the configured name of your servo
        Servo leftLink = hardwareMap.get(Servo.class, "leftLink"); // Replace "servo_name" with the configured name of your servo
        *//*

        Servo rightHorPivot = hardwareMap.get(Servo.class, "rightHorPivot"); // Replace "servo_name" with the configured name of your servo
        Servo leftHorPivot = hardwareMap.get(Servo.class, "leftHorPivot");
        Servo outtakePivot = hardwareMap.get(Servo.class, "outtakePivot");



        ArrayList<Motor> tempMotors = new ArrayList<Motor>();
        tempMotors.add(FL);
        tempMotors.add(FR);
        tempMotors.add(BL);
        tempMotors.add(BR);
        MotorGroup motors = new MotorGroup(tempMotors);

        motors.reset();
        motors.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);


        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.5, 1.75, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        BHI260IMU imu;
        Orientation angles;

        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                0,
                                0  // acquisitionTime, not used
                        )
                )
        );

        // Retrieve and initialize the IMU from the hardware map
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(myIMUparameters);

        telemetry.addData("Status", "BHI260 IMU initialized");
        telemetry.update();

        double OffsetYaw = imu.getRobotYawPitchRollAngles().getYaw();
        Boolean running = false;
        Boolean turning = false;
        Boolean negativeTurning = false;
        double angleF = 0;
        double Yaw = 0;
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        waitForStart();

        if (isStopRequested()) return;

        outtakeClaw.setPosition(0.3); // Set the servo to position 1
        outtakePivot.setPosition(0.0);

        intakeClaw.setPosition(1.0); // Set the servo to position 1
        rightHorPivot.setPosition(1.0);
        leftHorPivot.setPosition(0.0);

        while (opModeIsActive()) {

            if (gamepad2.a) {
                intakeClaw.setPosition(0.0); // Set the servo to position 0
                outtakeClaw.setPosition(0.7);
            }
            if (gamepad2.b) {
                intakeClaw.setPosition(1.0);
                outtakeClaw.setPosition(0.3);// Set the servo to position 1
            }
            */
/*
            if (gamepad2.x) {
                outtakeClaw.setPosition(0.3); // Set the servo to position 0
            }
            if (gamepad2.y) {
                outtakeClaw.setPosition(0.7); // Set the servo to position 1
            }*//*


            */
/*
            if (gamepad1.left_bumper){
                rightLink.setPosition(0.0);
                leftLink.setPosition(1.0);
            } else if (gamepad1.right_bumper){
                rightLink.setPosition(1.0);
                leftLink.setPosition(0.0);
            }*//*


            if (gamepad2.left_stick_y > 0.5){
                rightHorPivot.setPosition(0.35);
                leftHorPivot.setPosition(0.65);
            } else if (gamepad2.left_stick_y < -0.5){
                rightHorPivot.setPosition(1.0);
                leftHorPivot.setPosition(0.0);
            }

            if (gamepad2.right_stick_y > 0.5) {
                outtakePivot.setPosition(0.0);
            } else if (gamepad2.right_stick_y  < -0.5){
                outtakePivot.setPosition(1.0);
            }

            if (gamepad1.right_trigger > gamepad1.left_trigger){
                slideMotor.setPower(-gamepad1.right_trigger);
            } else {
                slideMotor.setPower(gamepad1.left_trigger);
            }

            // continuousServo.setPosition(0.0);


            if (!turning){
                robotOrientation = imu.getRobotYawPitchRollAngles();

                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double FLP = (y + x + rx) / denom;
                double BLP = (y - x + rx) / denom;
                double FRP = (y - x - rx) / denom;
                double BRP = (y + x - rx) / denom;

                FL.setPower(FLP * 0.6);
                FR.setPower(FRP * 0.6);
                BL.setPower(BLP * 0.6);
                BR.setPower(BRP * 0.6);


                telemetry.addData("Back Left Angle", BL.getDegrees());
                telemetry.addData("Front Left Angle", FL.getDegrees());
                telemetry.addData("Back Right Angle", BR.getDegrees());
                telemetry.addData("Front Right Angle", FR.getDegrees());

                telemetry.addData("Yaw", Yaw);

                telemetry.update();

                SparkFunOTOS.Pose2D pos = otos.getPosition();

                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);

                // Update the telemetry on the driver station
                telemetry.update();


                int degree = dpadButtons();
                if (degree > 0){
                    turning = true;
                    angleF = degree;
                }


                // telemetry.update();
            } else {
                for (double i = 1; i < 10; i++){
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                    double angleD = angleF - Yaw;
                    if(Math.abs(angleD) > 0.2) {
                        Boolean placeholder = turnTo( 1 / i, angleF, imu, OffsetYaw, motors, 0);
                    } else {
                        i = 10;
                    }

                }

                turning = false;
            */
/*
            turning = !turnTo(0.2, angleF, imu, OffsetYaw, motors, 0);
            turning = !turnTo(0.1, angleF, imu, OffsetYaw, motors, 0);
            *//*


            }
        }


    }


    public Boolean turnTo(double power, double angleF, IMU imu, Double OffsetYaw, MotorGroup motors, int run){
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
        if (0 > Yaw){
            Yaw+=360;
        }
        double angleD = angleF - Yaw;

        if (Math.abs(angleD) > 180){
            if (angleD > 0){
                angleD  = (360-Math.abs(angleD)) * -1;
            } else {
                angleD  = (360-Math.abs(angleD));
            }
        }

    */
/*

    if(Math.abs(angleD) < 2){
        return true;
    }
    *//*


        if (angleD < 0){
            power = power * -1; // right motor power first
        }

        if (angleD > 0){
            while (angleD > 0){
                for (Motor motor : motors.getMotors()) {
                    if (motor.getPosition().equals("right")) {
                        motor.setPower(power);
                    } else {
                        motor.setPower(power * -1);
                    }
                }
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }
                angleD = angleF - Yaw;

                if (Math.abs(angleD) > 180){
                    if (angleD > 0){
                        angleD  = (360-Math.abs(angleD)) * -1;
                    } else {
                        angleD  = (360-Math.abs(angleD));
                    }
                }
                telemetry.addData("angleF", angleF);
                telemetry.addData("Yaw", Yaw);
                telemetry.addData("angleD", Math.abs(angleD));
                telemetry.addData("runs", run);
                telemetry.addData("running", Math.random());

                telemetry.update();
            }
        } else {
            while (angleD < 0){
                for (Motor motor : motors.getMotors()) {
                    if (motor.getPosition().equals("right")) {
                        motor.setPower(power);
                    } else {
                        motor.setPower(power * -1);
                    }
                }
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
                if (0 > Yaw){
                    Yaw+=360;
                }
                angleD = angleF - Yaw;

                if (Math.abs(angleD) > 180){
                    if (angleD > 0){
                        angleD  = (360-Math.abs(angleD)) * -1;
                    } else {
                        angleD  = (360-Math.abs(angleD));
                    }
                }
                telemetry.addData("angleF", angleF);
                telemetry.addData("Yaw", Yaw);
                telemetry.addData("angleD", Math.abs(angleD));
                telemetry.addData("runs", run);
                telemetry.addData("running", Math.random());

                telemetry.update();
            }
        }

    */
/*
    robotOrientation = imu.getRobotYawPitchRollAngles();
    Yaw = (robotOrientation.getYaw(AngleUnit.DEGREES) - OffsetYaw) % 360;
    angleD = angleF - Yaw;

    if (run > 10){
        return true;
    }
    if(Math.abs(angleD) > 3) {
        return turnTo(power * 0.2, angleF, imu, OffsetYaw, motors, run + 1);
    }

    telemetry.addData("angleD", angleD);
    telemetry.addData("Yaw", Yaw);
    telemetry.addData("runs", run);
    telemetry.update();

     *//*


        return true;



    */
/*
    return turnTo(power * -0.8, angleF, imu, OffsetYaw, motors, run + 1);
    *//*


    }

    public int dpadButtons(){
        int degree = 0;
        if (gamepad1.dpad_up) {
            degree = 360;
            if (gamepad1.dpad_left){
                degree = 45;
            } else if (gamepad1.dpad_right){
                degree = 315;
            }
        } else if (gamepad1.dpad_left) {
            degree = 90;
            if (gamepad1.dpad_down){
                degree = 135;
            }
        } else if (gamepad1.dpad_down) {
            degree = 180;
            if (gamepad1.dpad_right){
                degree = 225;
            }
        } else if (gamepad1.dpad_right) {
            degree = 270;
        }
        return degree;
    }

}*/
