package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoFlipSubsystem extends SubsystemBase {
    private final Servo flipServo;
    private static final double POSITION_SPEC = 0.77; // Example position RIGHT
    private static final double POSITION_FORWARD = 0.38; // Example position LEFT

    private boolean isAtPositionForward = true;

    public ServoFlipSubsystem(HardwareMap hardwareMap) {
        flipServo = hardwareMap.get(Servo.class, "outtakeFlip");
        flipServo.setPosition(POSITION_SPEC); // Initialize at position A
    }

    public void moveToPositionSpec() {
        flipServo.setPosition(POSITION_SPEC);
        isAtPositionForward = true;
    }

    public void moveToPositionForward() {
        flipServo.setPosition(POSITION_FORWARD);
        isAtPositionForward = false;
    }

    public boolean isAtPositionRight() {
        return isAtPositionForward;
    }
}
