package org.firstinspires.ftc.teamcode.shhh.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Arm {

    private Servo pitch;
    private Claw claw;

    public enum State {
        back, up, down
    }

    public static double vert = 0.4, hori = 0;

    private State state = State.back, lastState = State.up;

    private boolean ispressed = false, ispressed2 = false;

    public static double aDown = 50, aUp = 168, aBack = 188;
    public static double aPitchUp=0.5, aPitchDown = 0.2, aPitchBack = 1;

    private double armDown = degToRange(aDown), armUp = degToRange(aUp), armBack = degToRange(aBack);

    public Arm(HardwareMap hardwareMap) {
        pitch = hardwareMap.servo.get("pitch");
        pitch.setDirection(Servo.Direction.REVERSE);
        claw = new Claw(hardwareMap);

        pitch.setPosition(0.2);
    }

    int incr = 2; boolean lock = false; int lincr = 0;


    public void process(boolean down, boolean up) {
        if (down && !lock && incr > 0) {
            incr -= 1;
            lock = true;
        } else if (up && !lock && incr < 2) {
            incr += 1;
            lock = true;
        } else if (!down && !up) {
            lock = false;
        }
    }

    public void update(boolean wristT, boolean clawT) {
        claw.update(clawT);

        if (incr != lincr) {
            if (incr == 1) {
                pitch.setPosition(0.5);
            } else if (incr == 0) {
                pitch.setPosition(0.2);
            } else {
                pitch.setPosition(1);
            }
            lincr = incr;
        }
    }

    public static double degToRange(double deg) {
        return Range.clip(deg, 0, 300)/300;
    }


    public void setDeposit(double wristp, double pitchp, double armp, double clawp) {
        pitch.setPosition(pitchp); claw.directSet(clawp);
    }

}
