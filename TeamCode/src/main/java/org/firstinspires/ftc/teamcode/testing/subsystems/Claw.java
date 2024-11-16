package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;

    public static double open = 0, close = 0.5;

    private boolean isPressed = false;

    public Claw(HardwareMap hwmap, String deviceName)
    {
        claw = hwmap.servo.get(deviceName);
        claw.setPosition(close);
    }

    public void update(boolean input)
    {
        if (input && !isPressed)
        {
            if (claw.getPosition() == open)
            {
                claw.setPosition(close);
            }
            else
            {
                claw.setPosition(open);
            }
        }
        isPressed = input;
    }

    public void setPos(double pos)
    {
        claw.setPosition(pos);
    }


}
