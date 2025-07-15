package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.SparkFunOTOSDrive;


@Config
@Autonomous(name = "IntoTheDeep", group = "Autonomous")
public class Auto extends LinearOpMode {


    public class LiftC {
        private DcMotorEx lift;

        public LiftC(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -600.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -2) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }


    public class Claw {
        private Servo clawObj;
        private Servo clawPivotObj;

        public Claw(HardwareMap hardwareMap){
            clawObj = hardwareMap.get(Servo.class, "outtakeClaw");
            clawPivotObj = hardwareMap.get(Servo.class, "outtakePivot");
        }

        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawObj.setPosition(0.7); // Set the servo to position 1
                return false;
            }
        }

        public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawObj.setPosition(0.5); // Set the servo to position 1
                return true;
            }
        }

        public Action openClaw(){
            return new openClaw();
        }

        public Action closeClaw(){
            return new closeClaw();
        }

        public class pivotClawDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPivotObj.setPosition(0.0);
                return true;
            }
        }
        public class pivotClawUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPivotObj.setPosition(0.5); // Set the servo to position 1
                return true;
            }
        }
        public Action pivotClawDown(){
            return new pivotClawDown();
        }

        public Action pivotClawUp(){
            return new pivotClawUp();
        }


    }
    // outtake servo 0.5 close 1 open
    // pivot outtake 1 place 0 intake
    // left pivot - left is up right is down
    // intake 0.5 close right open
    @Override
    public void runOpMode() {
        Claw outtakeClaw = new Claw(hardwareMap);

        LiftC outtakeSlide = new LiftC(hardwareMap);

        Pose2d initialPose = new Pose2d(0, -61.5, Math.toRadians(180));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        // vision here that outputs position

        TrajectoryActionBuilder goToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -34))
                .waitSeconds(1);

        TrajectoryActionBuilder goToHP = goToBar.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(34, -55), Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder moveForwardToGrab = goToHP.endTrajectory().fresh()
                .strafeTo(new Vector2d(34, -57))
                .waitSeconds(1);

        TrajectoryActionBuilder goBackToBar = moveForwardToGrab.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0, -34), Math.toRadians(-90))
                .waitSeconds(1);

        Action goToObservation = goBackToBar.endTrajectory().fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(56, -61.5))
                .build();
/*
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

*/


        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        goToBar.build()
                )
        );


    }
}