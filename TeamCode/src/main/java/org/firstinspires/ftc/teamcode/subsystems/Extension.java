package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.HashMap;

@Config
public class Extension {

    private DcMotor leftExtension, rightExtension;

    public static double power = 0.5;

    public String posStr = "";

    public static int specIntake = 1000, sampleIntake = 5, max = 2350, highBasket = 2350, lowBasket = 100, lowSpec = 5, highSpec = 1000, idle = 5;
    private int pos;

    public static double kP = 0.005, kI = 0, kD = 0;

    private int curLeft;

    PIDController pidController = new PIDController(kP, kI, kD);

    RevTouchSensor reset;
    public Extension(HardwareMap hwMap, HashMap<String, String> config)
    {
        leftExtension = hwMap.dcMotor.get(config.get("leftExtension"));
        rightExtension = hwMap.dcMotor.get(config.get("rightExtension"));

        //reset = hwMap.get(RevTouchSensor.class, config.get("reset"));

        rightExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        pidController.setTolerance(10);

    }

    public void update()
    {
        curLeft = leftExtension.getCurrentPosition();

        if (check(curLeft, pos, 10))
        {
            applyPower(0);
        }
        else{
            applyPower(pidController.calculate(curLeft, pos));
        }
    }

    public void setPos(String pos)
    {
        switch (pos)
        {
            case "Specimen Intake":
                this.pos = specIntake;
                break;

            case "Sample Intake":
                this.pos = idle;
                break;

            case "Low Basket":
                this.pos = lowBasket;
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

            default:
                this.pos = idle;
                break;
        }

        posStr = pos;

    }

    public void applyPower(double power)
    {
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public boolean isBusy(){
        return leftExtension.isBusy();
    }

    public String getTarget()
    {
        return posStr;
    }

    /*public void checkReset()
    {
        if(reset.isPressed())
        {
            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }*/

    public boolean check(int cur, int target, int thresh)
    {
        return Math.abs(cur - target) < thresh;
    }
    public int getError()
    {
        return pos - curLeft;
    }

}
