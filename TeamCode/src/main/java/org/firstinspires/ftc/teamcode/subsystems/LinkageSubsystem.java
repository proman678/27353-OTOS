package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LinkageSubsystem extends SubsystemBase {
    private final Servo rightLinkageServo;
    private final Servo leftLinkageServo;
    private static final double RIGHT_IN_POSITION = 0.8; // Example position RIGHT
    private static final double RIGHT_OUT_POSITION = 0.2; // Example position LEFT
    private static final double LEFT_IN_POSITION = 0;
    private static final double LEFT_OUT_POSITION = 1;

    private boolean isAtPositionOut = true;

    public LinkageSubsystem(HardwareMap hardwareMap) {
        rightLinkageServo = hardwareMap.get(Servo.class, "rightLinkageServo");
        rightLinkageServo.setPosition(RIGHT_IN_POSITION); // Initialize at position A
        leftLinkageServo = hardwareMap.get(Servo.class, "leftLinkageServo");
        leftLinkageServo.setPosition(LEFT_IN_POSITION); // Initialize at position A
    }

    public void moveToPositionOut() {
        rightLinkageServo.setPosition(RIGHT_OUT_POSITION);
        isAtPositionOut = true;
        leftLinkageServo.setPosition(LEFT_OUT_POSITION);
    }

    public void moveToPositionIn() {
        rightLinkageServo.setPosition(RIGHT_IN_POSITION);
        isAtPositionOut = false;
        leftLinkageServo.setPosition(LEFT_IN_POSITION);
    }

    public boolean isAtPositionOut() {
        return isAtPositionOut;
    }
}
