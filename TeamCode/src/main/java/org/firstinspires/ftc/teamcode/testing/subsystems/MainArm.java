package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MainArm {

    private PivotMP pivot; private Lift lift;

    public enum State
    {
        intake,
        basket,
        backpickup,
        idle
    }

    public static double vertAngle = 90, backUpAngle = 110, specimen = 70;

    private boolean flag = false;

    public MainArm(HardwareMap hardwareMap,String rightPivot, String leftPivot, String leftMotor, String rightMotor, String touchSensor, double PIDTol, boolean manual)
    {
        pivot = new PivotMP(hardwareMap, rightPivot, leftPivot); lift = new Lift(hardwareMap, leftMotor, rightMotor, touchSensor, PIDTol, manual);
        flag = manual;
        pivot.setTargetAngle(0); lift.setTarget(0); lift.setLimit(false);
    }

    private State mainState = MainArm.State.intake, lState = mainState;

    public void setMainState(State mainState)
    {
        this.mainState = mainState;
    }

    private double lmi = 0;

    public void setLmi(double lmi) {
        this.lmi = lmi;
    }

    private double ltg = 0;

    public void setLtg(double ltg) {
        this.ltg = ltg;
    }

    public void update( boolean liftReset)
    {
        if (mainState != lState)
        {
            switch (mainState)
            {
                case basket:
                    pivot.setTargetAngle(vertAngle);
                    //lift.setLimit(true);
                    break;
                case intake:
                    pivot.setTargetAngle(0);
                    //lift.setLimit(false);
                    break;
                case idle:
                    pivot.setTargetAngle(specimen);
                    //lift.setLimit(true);
                    break;
                case backpickup:
                    pivot.setTargetAngle(backUpAngle);
                    //lift.setLimit(true);
                    break;
            }
        }

        lState = mainState;

        if (flag)
        {
            lift.setManualInput(lmi);
        }
        else
        {
            lift.setTarget(ltg);
        }

        pivot.update();
        lift.update(liftReset);
    }
    public State getlState() {
        return lState;
    }
    public State getState() {
        return mainState;
    }

    public double getAngle() {
        return pivot.angle;
    }
    public double getTAngle() {
        return pivot.targetAngle;
    }
    public double getPower() {
        return pivot.power;
    }


    public boolean stateComplete() {
        return lift.check() && pivot.check();
    }
}
