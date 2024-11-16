package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotor lLift, rLift;
    //private RevTouchSensor reset;

    private double ticks, power, lastPower;
    private double target = 0;

    public static double p = 0.01, i, d;

    public PIDController controller = new PIDController(p, i , d);

    private boolean manual = false;
    private double mInput = 0;

    private double pos1 = 2592, pos2 = 1600, limit = pos2; // TODO: change the tick values

    public Lift(HardwareMap hwMap, String leftMotor, String rightMotor, String touchSensor, double PIDTol, boolean manual)
    {
        lLift = hwMap.dcMotor.get(leftMotor);
        rLift = hwMap.dcMotor.get(rightMotor);

        lLift.setDirection(DcMotorSimple.Direction.REVERSE);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //reset = hwMap.get(RevTouchSensor.class, touchSensor);

        controller.setTolerance(PIDTol);

        this.manual = manual;
    }

    public double readTicks(double ticks, boolean flag)
    {
        if (/*reset.isPressed()*/ flag)
        {
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return 0;
        }
        return ticks;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setManualInput(double input) {
        mInput = input;
    }

    public void setLimit(boolean isVertical) {
        if (isVertical) {
            limit = pos1;
        } else {
            limit = pos2;
        }
    }

    public void update(boolean flag) {
        ticks = readTicks(Math.abs(lLift.getCurrentPosition()), flag);
        if (manual) {
            double dt = 45*mInput;
            target += dt;
        }

        if (target >= limit) {
            target = limit;
        } else if (target <= 0) {
            target = 0;
        }

        controller.setSetPoint(target);

        power = controller.calculate(ticks);

        if (Utils.valInThresh(power, lastPower, 0.001)) {
            if (target == 0 && flag) {
                power = 0;
            }
            apply(power);
            lastPower = power;
        }
    }

    public double getTicks() {
        return ticks;
    }

    private void apply(double p) {
        lLift.setPower(p * 0.1);
        rLift.setPower(p * 0.1);
    }

    public boolean check() {
        return controller.atSetPoint();
    }

}
