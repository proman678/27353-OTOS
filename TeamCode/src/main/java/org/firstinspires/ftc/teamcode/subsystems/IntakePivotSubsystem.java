package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakePivotSubsystem extends SubsystemBase {
    private final Servo rightHorPivot;
    private final Servo leftHorPivot;
    private static final double RIGHT_POSITION_UP = 0.22; // Example position RIGHT
    private static final double RIGHT_POSITION_DOWN = 0.06; // Example position LEFT
    private static final double LEFT_POSITION_UP = 0;
    private static final double LEFT_POSITION_DOWN = 0.26;

    private boolean isAtPositionUp = true;

    public IntakePivotSubsystem(HardwareMap hardwareMap) {
        rightHorPivot = hardwareMap.get(Servo.class, "rightIntakeServo");
        rightHorPivot.setPosition(RIGHT_POSITION_UP); // Initialize at position A
        leftHorPivot = hardwareMap.get(Servo.class, "leftIntakeServo");
        leftHorPivot.setPosition(LEFT_POSITION_UP); // Initialize at position A*/
    }

    public void moveToPositionUp() {
        rightHorPivot.setPosition(RIGHT_POSITION_UP);
        isAtPositionUp = true;
        leftHorPivot.setPosition(LEFT_POSITION_UP);
    }

    public void moveToPositionDown() {
        rightHorPivot.setPosition(RIGHT_POSITION_DOWN);
        isAtPositionUp = false;
        leftHorPivot.setPosition(LEFT_POSITION_DOWN);
    }

    public boolean isAtPositionUp() {
        return isAtPositionUp;
    }
}
