package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    private Servo tilt;

    public Tilt(HardwareMap hwMap) {
        tilt = hwMap.get(Servo.class, "tilt");                                       // TODO: Set to actual hwMap deviceName
    }

    public class TiltZero implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            tilt.setPosition(0.5);                                                              // TODO: Set to actual tilt position
            return false;
        }
    }

    public class ClawForward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            tilt.setPosition(0);                                                                // TODO: Set to actual tilt position
            return false;
        }
    }

    public class SpinnerForward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            tilt.setPosition(1);                                                                // TODO: Set to actual tilt position
            return false;
        }
    }

    public class TiltPosition implements Action {
        float tiltPos;
        public boolean run(@NonNull TelemetryPacket packet) {
            tilt.setPosition(tiltPos);
            return false;
        }
    }

    public class TiltPreset implements Action {
        String preset;
        public boolean run(@NonNull TelemetryPacket packet) {
            if (preset.equals("Bucket High"))
            {
                tilt.setPosition(1); // TODO
            }
            else if (preset.equals("Bucket Low"))
            {
                tilt.setPosition(1); // TODO
            }
            else if (preset.equals("Bar High"))
            {
                tilt.setPosition(1); // TODO
            }
            else if (preset.equals("Bar Low"))
            {
                tilt.setPosition(1); // TODO
            }
            else if (preset.equals("Floor"))
            {
                tilt.setPosition(1); // TODO
            }

            return false;
        }
    }

    public Action zero() {
        return new Tilt.TiltZero();
    }

    public Action clawForward() {
        return new Tilt.ClawForward();
    }

    public Action spinnerForward() {
        return new Tilt.SpinnerForward();
    }

    public Action tiltPosition(float pos) {
        Tilt.TiltPosition tiltAction = new Tilt.TiltPosition();
        tiltAction.tiltPos = pos;
        return tiltAction;
    }

    public Action tiltPreset(String pos) {
        Tilt.TiltPreset tiltAction = new Tilt.TiltPreset();
        tiltAction.preset = pos;
        return tiltAction;
    }
}