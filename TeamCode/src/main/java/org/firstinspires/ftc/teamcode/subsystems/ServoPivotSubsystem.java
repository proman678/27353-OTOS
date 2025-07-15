package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPivotSubsystem extends SubsystemBase {
    private final Servo pivotServo;
    private static final double POSITION_UP = 0.4; // Example position RIGHT
    private static final double POSITION_DOWN = 0.25; // Example position LEFT

    private boolean isAtPositionUp = true;

    public ServoPivotSubsystem(HardwareMap hardwareMap) {
        pivotServo = hardwareMap.get(Servo.class, "outtakePivot");
        pivotServo.setPosition(POSITION_DOWN); // Initialize at position A
    }

    public void moveToPositionUp() {
        pivotServo.setPosition(POSITION_UP);
        isAtPositionUp = true;
    }

    public void moveToPositionDown() {
        pivotServo.setPosition(POSITION_DOWN);
        isAtPositionUp = false;
    }

    public boolean isAtPositionRight() {
        return isAtPositionUp;
    }
}
