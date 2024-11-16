package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    public Claw(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");                                       // TODO: Set to actual hwMap deviceName
    }

    // Define Actions
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) { // This makes the method run while it returns true and is always needed idk
            claw.setPosition(1);                                                                // TODO: Set to actual claw position
            return false; // Makes method run once
        }
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(0);                                                                // TODO: Set to actual claw position
            return false;
        }
    }

    // Wrap Actions as functions
    public Action closeClaw() {
        return new Claw.CloseClaw();
    }

    public Action openClaw() {
        return new Claw.OpenClaw();
    }
}
