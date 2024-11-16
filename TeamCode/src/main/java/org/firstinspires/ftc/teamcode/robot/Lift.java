package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

// Class for lift Actions
public class Lift {

    // test
    private DcMotorEx leftPivot;
    private DcMotorEx rightPivot;

    private DcMotorEx leftExtension;
    private DcMotorEx rightExtension;

    public Lift(HardwareMap hwMap) {
        leftPivot = hwMap.get(DcMotorEx.class, "leftPivot");                             // TODO: Set to actual hwMap deviceName
        rightPivot = hwMap.get(DcMotorEx.class, "rightPivot");                           // TODO: Set to actual hwMap deviceName
        leftExtension = hwMap.get(DcMotorEx.class, "leftExtension");                     // TODO: Set to actual hwMap deviceName
        rightExtension = hwMap.get(DcMotorEx.class, "rightExtension");                   // TODO: Set to actual hwMap deviceName

        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                            // TODO: Set to actual hwMap deviceName
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                           // TODO: Set to actual hwMap deviceName
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                        // TODO: Set to actual hwMap deviceName
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                       // TODO: Set to actual hwMap deviceName


    }

    // TODO: Define Actions
    public class ArmPreset implements Action {
        String preset;
        public boolean run(@NonNull TelemetryPacket packet) {
            if (preset.equals("Bucket High"))
            {
                // TODO
            }
            else if (preset.equals("Bucket Low"))
            {
                // TODO
            }
            else if (preset.equals("Bar High"))
            {
                // TODO
            }
            else if (preset.equals("Bar Low"))
            {
                // TODO
            }
            else if (preset.equals("Floor"))
            {
                // TODO
            }

            return false;
        }
    }

    // TODO: Wrap Actions as functions
    public Action armPreset(String pos) {
        Lift.ArmPreset liftAction = new Lift.ArmPreset();
        liftAction.preset = pos;
        return liftAction;
    }

}
