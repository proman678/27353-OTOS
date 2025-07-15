package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;

    private boolean isOn = false; // Track the current state

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setPower(0); // Initialize in the closed position
    }

    public void on() {
        intakeServo.setPower(-1);
        isOn = true;
    }

    public void off() {
        intakeServo.setPower(0);
        isOn = false;
    }

    public void reverse() {
        intakeServo.setPower(1);
        isOn = true;
    }


    public boolean isOn() {
        return isOn;
    }
}
