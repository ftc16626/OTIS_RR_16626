package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {

    private Servo claw;
    private boolean ispressed = false;
    public static double closed = 0.55, open = 0.21;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(open);
    }

    public void update(boolean gp) {
        if (gp && !ispressed) {
            if (claw.getPosition() == closed) {
                claw.setPosition(open);
            } else {
                claw.setPosition(closed);
            }
        } ispressed = gp;
    }

    public void directSet(double p) {
        claw.setPosition(p);
    }

}
