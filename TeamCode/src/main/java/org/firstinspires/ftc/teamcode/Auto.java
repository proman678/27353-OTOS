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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



@Config
@Autonomous(name = "IntoTheDeep", group = "Autonomous")
public class Auto extends LinearOpMode {

    public class Claw {
        private Servo clawObj;

        public Claw(HardwareMap hardwareMap){
            clawObj = hardwareMap.get(Servo.class, "outtakeClaw");
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

    }
    // outtake servo 0.5 close 1 open
    // pivot outtake 1 place 0 intake
    // left pivot - left is up right is down
    // intake 0.5 close right open
    @Override
    public void runOpMode() {
        Claw outtakeClaw = new Claw(hardwareMap);
        Pose2d initialPose = new Pose2d(0, -61.5, Math.toRadians(-90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        // vision here that outputs position

        TrajectoryActionBuilder goToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -34));

        Action goToObservation = goToBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(56, -61.5))
                .build();
/*
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
*/

        Servo clawObj = hardwareMap.get(Servo.class, "outtakeClaw");
        clawObj.setPosition(0.3);
        // Actions.runBlocking(outtakeClaw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Action trajectoryActionChosen = goToBar.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        outtakeClaw.openClaw(),
                        goToObservation
                )
        );
    }
}