package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDTune")
public class PIDTuning extends OpMode {
    private PIDFController armcontroller;
    private PIDFController slidecontroller;
    public static double Arm_p = 0, Arm_i = 0, Arm_d = 0;
    public static double Arm_f = 0;
    public static double armDegrees = 0;
    public static double Slide_p = 0, Slide_i = 0, Slide_d = 0;
    public static double Slide_f = 0;
    public static double slideInches = 0;
    final static double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
    final static double COUNTS_PER_DEGREE = COUNTS_PER_ROT_MOTOR_REV / 180;
    final static double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
    final static double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
    final static double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
            (PULLEY_DIAMETER_INCHES * 3.1415);
    public static int armtarget = (int)(armDegrees * COUNTS_PER_DEGREE);
    public static int slidetarget = (int)(slideInches * COUNTS_PER_EXTINCH);
    private DcMotorEx rotateArm;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;

    @Override
    public void init() {
        armcontroller = new PIDFController (Arm_p,Arm_i,Arm_d,Arm_f);
        slidecontroller = new PIDFController (Slide_p,Slide_i,Slide_d,Slide_f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rotateArm = hardwareMap.get(DcMotorEx.class,"rotateArm");
        extendArm1 = hardwareMap.get(DcMotorEx.class,"extendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class,"extendArm2");

    }

    @Override
    public void loop() {
        armcontroller.setPIDF(Arm_p,Arm_i,Arm_d,Arm_f);
        slidecontroller.setPIDF(Slide_p,Slide_i,Slide_d,Slide_f);
        int armpos = rotateArm.getCurrentPosition();
        int slidepos1 = extendArm1.getCurrentPosition();
        int slidepos2 = extendArm2.getCurrentPosition();
        double armpid = armcontroller.calculate(armpos, armtarget);
        double slidepid1 = slidecontroller.calculate(slidepos1,slidetarget);
        double slidepid2 = slidecontroller.calculate(slidepos2,slidetarget);
        double rot_power = armpid;
        double slide_power1 = slidepos1;
        double slide_power2 = slidepos2;
        rotateArm.setPower(rot_power);
        extendArm1.setPower(slide_power1);
        extendArm2.setPower(slide_power2);

        telemetry.addData("armpos", armpos);
        telemetry.addData("armtarget", armtarget);
        telemetry.addData("slidepos1",slidepos1);
        telemetry.addData("slidepos2",slidepos2);
        telemetry.addData("slidetarget",slidetarget);
        telemetry.update();



    }
}