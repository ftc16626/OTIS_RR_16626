package org.firstinspires.ftc.teamcode.subsystems;

import android.text.InputFilter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Util;

import java.util.HashMap;

@Config
public class Pivot {

    private DcMotorEx leftPivot, rightPivot;

    public static double power = 0, lastPower = power;

    public static int intake = 30, max = 325, highBasket = 300, lowSpec = 120, highSpec = 200, idle = 300, zero = 0;
    private int pos;

    public static double kP = 0.01, kI = 0, kD = 0, k = 0;

    public static double extendedKp = 0.05, zeroKp = 0.005;

    PIDController pidController = new PIDController(kP, kI, kD);

    private int curLeft = 0, langle = curLeft;

    public static int threshold = 10;

    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Util.MAX_PIVOT_VELOCITY, Util.MAX_PIVOT_ACCEL);

    private int lta = 0;

    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    double aVelocity, indexedPosition = 0;

    RevTouchSensor reset;

    public Pivot(HardwareMap hwMap, HashMap<String, String> config)
    {
        leftPivot = hwMap.get(DcMotorEx.class,config.get("leftPivot"));
        rightPivot = hwMap.get(DcMotorEx.class,config.get("rightPivot"));

        rightPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset = hwMap.get(RevTouchSensor.class, config.get("reset"));

        profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
    }

    public void update()
    {
        
        curLeft = leftPivot.getCurrentPosition();
        //curRight = rightPivot.getCurrentPosition();
        //checkReset();
        if (pos != lta) {
            profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(pos, 0), new TrapezoidProfile.State(curLeft, aVelocity));
            fullTimer.reset();
        }

        indexedPosition = profile.calculate(fullTimer.seconds()).position;

        pidController.setSetPoint(indexedPosition);

        power = pidController.calculate(curLeft) + (k * Math.cos(pos));
        
        if (Util.inThresh(power, lastPower, 0.001)) {
            applyPower(power);
            lastPower = power;
        }
        

        lta = pos;

        aVelocity = (curLeft-langle)/velTimer.seconds();
        langle = curLeft;
        velTimer.reset();
    }

    public void applyPower(double power)
    {
        leftPivot.setPower(power);
        rightPivot.setPower(power);
    }

    public void changePos(int tickChange)
    {
        pos = leftPivot.getCurrentPosition() + tickChange;
    }

    public void setPos(String pos)
    {
        switch (pos)
        {
            case "Specimen Intake":
                this.pos = intake;
                break;

            case "Sample Intake":
                this.pos = intake;
                break;

            case "Low Basket":
                this.pos = highBasket;
                break;

            case "High Basket":
                this.pos = highBasket;
                break;

            case "Low Specimen":
                this.pos = lowSpec;
                break;

            case "High Specimen":
                this.pos = highSpec;
                break;

            case "Zero":
                this.pos = zero;
                break;
            

            default:
                this.pos = idle;
                break;
        }
    }

    public void setKP(String pos)
    {
        switch(pos)
        {
            case "Specimen Intake":
                pidController.setP(kP);
                break;

            case "Sample Intake":
                pidController.setP(kP);
                break;

            case "Low Basket":
                pidController.setP(extendedKp);
                break;

            case "High Basket":
                pidController.setP(extendedKp);
                break;

            case "Low Specimen":
                pidController.setP(kP);
                break;

            case "High Specimen":
                pidController.setP(extendedKp);
                break;
            case "Zero":
                pidController.setP(zeroKp);
                break;

            case "High Spec Depo":
                pidController.setP(extendedKp);
                break;
                
            default:
                pidController.setP(kP);
                break;
        }
    }
    public boolean check(int cur, int target, int thresh)
    {
        return Math.abs(target - cur) < thresh;
    }

    public int getError()
    {
        return pos - curLeft;
    }

    public boolean isBusy() {
        return leftPivot.isBusy();
    }

    public int getTicks() {
        return leftPivot.getCurrentPosition();
    }

    public double getKP()
    {
        return pidController.getP();
    }

    public void reset(){
        setPos("Zero");
        setKP("Zero");
        update();
        checkReset();

    }

    public boolean checkReset()
    {
        if (reset.isPressed())
        {
            curLeft = 0;
            leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return reset.isPressed();
    }

    public void setDirectPos(int pos)
    {
        this.pos = pos;
    }

    public void setDirectKP(double Kp)
    {
        pidController.setP(kP);
    }

    public int getPos()
    {
        return leftPivot.getCurrentPosition();
    }

}