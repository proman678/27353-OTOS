package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClawSubsystem extends SubsystemBase {
    private final Servo clawServo;

    // Define positions
    private static final double CLOSE_POSITION = 0.4;
    private static final double OPEN_POSITION = 0.7;

    private boolean isClosed = true; // Track the current state

    public ServoClawSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "outtakeClaw");
        clawServo.setPosition(CLOSE_POSITION); // Initialize in the closed position
    }

    public void close() {
        clawServo.setPosition(CLOSE_POSITION);
        isClosed = true;
    }

    public void open() {
        clawServo.setPosition(OPEN_POSITION);
        isClosed = false;
    }

    public boolean isClosed() {
        return isClosed;
    }
}
