package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

@Config
public class Drive {

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    private double x, y, rx, d;

    public static double speedMult = 0.8, rotMult = 0.5, slowMult = 0.5, curveFactor = 1.5, strafeCorrection = 1.1;

    private boolean slowMode = false;

    public Drive(HardwareMap hwMap, HashMap<String, String> config)
    {
        frontLeft = hwMap.dcMotor.get(config.get("frontLeft"));
        backLeft = hwMap.dcMotor.get(config.get("backLeft"));
        frontRight = hwMap.dcMotor.get(config.get("frontRight"));
        backRight = hwMap.dcMotor.get(config.get("backRight"));

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void getXYZ(double x, double y, double rx)
    {
        this.x =Math.signum(x) * Math.pow(Math.abs(x * speedMult), curveFactor) * strafeCorrection;
        this.y = Math.signum(-y) * Math.pow(Math.abs(y * speedMult), curveFactor);
        this.rx = Math.signum(-rx) * Math.pow(Math.abs(rx * rotMult), curveFactor);
    }

    public void update()
    {
        d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / d);
        backLeft.setPower((y - x + rx) / d);
        frontRight.setPower((y - x - rx) / d);
        backRight.setPower((y + x - rx) / d);
    }

    public void accelerateForward(double u, double v, double millis)
    {
        for (int i = 0; i < millis; i ++)
        {
            applyPower(u + ((v-u)/millis) * i);
            try {
                Thread.sleep(1); // Wait for 1 millisecond
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Restore interrupted status
                // Optionally, log or handle the interruption
            }
        }
    }

    public void applyPower(double p)
    {
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }


    public void toggleSlowMode()
    {
        if (slowMode)
        {
            slowMode = false;
            speedMult /= slowMult;
            rotMult /= slowMult;
        }
        else if (!slowMode)
        {
            slowMode = true;
            speedMult *= slowMult;
            rotMult *= slowMult;
        }
    }
}
