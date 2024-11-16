package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Util {
    public static double MAX_PIVOT_VELOCITY = 3000; // 4pi rad/sec
    public static double MAX_PIVOT_ACCEL = 2000; // 2pi rad/sec^2

    public static boolean inThresh(double val, double val2, double tol) {
        return Math.abs(val - val2) > tol;
    }
}
