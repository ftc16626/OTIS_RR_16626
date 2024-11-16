package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {
    private CRServo spinner;

    public Spinner(HardwareMap hwMap) {
        spinner = hwMap.get(CRServo.class, "spinner");                               // TODO: Set to actual hwMap deviceName
    }

    // Define Actions
    public class SpinnerOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            spinner.setPower(1);
            return false;
        }
    }

    public class SpinnerOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            spinner.setPower(0);
            return false;
        }
    }

    public class SpinnerReverse implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            spinner.setPower(-1);
            return false;
        }
    }

    // Wrap Actions as functions
    public Action spinnerOn() {
        return new Spinner.SpinnerOn();
    }

    public Action spinnerOff() {
        return new Spinner.SpinnerOff();
    }

    public Action spinnerReverse() {
        return new Spinner.SpinnerReverse();
    }
}