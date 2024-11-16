package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class PivotMP {

    private DcMotor lPivot, rPivot;

    private RevTouchSensor reset;

    public static double p = 3, i = 0, d = 0.1, k = 0.2;
    PIDController controller = new PIDController(p, i, d);

    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.1*Math.PI, 2*Math.PI);

    public double tpr = (537.7*(60/24))/(2*Math.PI); // motor tick rate * (input gear/output gear)/2pi

    public boolean resetFlag = false;

    public static double vertAngle = 90, backUpAngle = 110, specimen = 70;

    public double updateCurrentAngle(double current) { // if slide reset, return 0, otherwise, return angle in rad
        if (reset.isPressed()) {
            rPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetFlag = true;
            return  0;
        } else {
            resetFlag = false;
            return  current/tpr;
        }
    }

    public double angle = 0, langle = angle, aVelocity = 0; // langle is last angle, aVelocity is ang. vel.

    public PivotMP(HardwareMap hardwareMap, String rightPivot, String leftPivot) {
        rPivot = hardwareMap.dcMotor.get(rightPivot);
        lPivot = hardwareMap.dcMotor.get(leftPivot);

        rPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        lPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset = hardwareMap.get(RevTouchSensor.class, "reset");

        profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
    }

    public double targetAngle = 0, power = 0, lastPower = power, lta = targetAngle, indexedPosition = 0; // lta is last target angle

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = Math.toRadians(targetAngle);
    }


    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    public void update() {

        angle = Math.abs(updateCurrentAngle(rPivot.getCurrentPosition()));

        if (targetAngle != lta) {
            profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(targetAngle, 0), new TrapezoidProfile.State(angle, aVelocity));
            fullTimer.reset();
        }

        indexedPosition = profile.calculate(fullTimer.seconds()).position;

        controller.setSetPoint(indexedPosition);

        power = controller.calculate(angle) + (k * Math.cos(targetAngle));

        if (Utils.valInThresh(power, lastPower, 0.001)) {
            if (indexedPosition == 0 && resetFlag) {
                power = 0;
            }
            apply(power);
            lastPower = power;
        }

        lta = targetAngle;

        aVelocity = (angle-langle)/velTimer.seconds();
        langle = angle;
        velTimer.reset();
    }

    public void apply(double p) {
        rPivot.setPower(-p);
        lPivot.setPower(-p);
    }

    public boolean check() {
        return Math.abs(Math.toDegrees(targetAngle)-Math.toDegrees(angle)) < 5;
    }
}

