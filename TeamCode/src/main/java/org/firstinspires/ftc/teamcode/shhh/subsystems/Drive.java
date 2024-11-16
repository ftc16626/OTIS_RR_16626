package org.firstinspires.ftc.teamcode.shhh.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {

    private DcMotor fl, fr, bl, br;
    private double x, y, z, d; // z is rotation, d is for normalization
    private double gpx, gpy, gpz; // gpx = gamepad x input, gpy = gamepad y input, gpz = gamepad rot input

    private boolean isPr = false, downFlag = false, upFlag = false; // isPr makes sure that mult is not changed twice per update() call. upFlag and downFlag are for changing speed mult

    private double mult = 0.8; // speed mult

    public Drive(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("frontLeftMotor");
        bl = hardwareMap.dcMotor.get("backLeftMotor");
        fr = hardwareMap.dcMotor.get("frontRightMotor");
        br = hardwareMap.dcMotor.get("backRightMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setFlags(boolean down, boolean up) {
        upFlag = up; downFlag = down;
    }

    public void setXYZ(double x, double y, double z) {
        gpx = -x; gpy = -y; gpz = -z;
    }

    public void update() {
        if (downFlag && !isPr && mult > 0.1) {
            mult -= 0.1;
            isPr = true;
        } else if (upFlag && !isPr && mult < 1) {
            mult += 0.1;
            isPr = true;
        } else if (!downFlag && !upFlag) {
            isPr = false;
        }

        x = -gpx; y = gpy; z = gpz;
        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        smp(x * mult, y * mult, z * mult);
    }

    public double getMultiplier() {
        return mult;
    }

    private double abs(double i) {
        return Math.abs(i);
    }

    public void smp(double x, double y, double z) { // calculate power per motor
        fl.setPower((y + x - z)/d);
        bl.setPower((y - x - z)/d);
        fr.setPower((y - x + z)/d);
        br.setPower((y + x + z)/d);
    }

}
