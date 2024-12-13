package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPivotSubsystem extends SubsystemBase {
    private final Servo pivotServo;
    private static final double POSITION_RIGHT = 0.8; // Example position RIGHT
    private static final double POSITION_LEFT = 0.1; // Example position LEFT
    private boolean isAtPositionRight = true;

    public ServoPivotSubsystem(HardwareMap hardwareMap) {
        pivotServo = hardwareMap.get(Servo.class, "outtakePivot");
        pivotServo.setPosition(POSITION_RIGHT); // Initialize at position A
    }

    public void moveToPositionRight() {
        pivotServo.setPosition(POSITION_RIGHT);
        isAtPositionRight = true;
    }

    public void moveToPositionLeft() {
        pivotServo.setPosition(POSITION_LEFT);
        isAtPositionRight = false;
    }

    public boolean isAtPositionRight() {
        return isAtPositionRight;
    }
}
