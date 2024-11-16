/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import java.util.HashMap;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@TeleOp(name = "preload specimen only")
@Config
public class PreloadSpec extends LinearOpMode {
    public static double u = 0, v = 0.4;
    public static int t = 200;
    public HashMap<String, String> deviceConf = new HashMap<String, String>();

    public static int tickChange = 50;

    public static double clawOpen = 0.21, clawClose = 0.55;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot Config Mapping
        deviceConf.put("frontLeft",       "frontLeftMotor");
        deviceConf.put("backLeft",        "backLeftMotor");
        deviceConf.put("frontRight",      "frontRightMotor");
        deviceConf.put("backRight",       "backRightMotor");
        deviceConf.put("leftPivot",       "leftPivot");
        deviceConf.put("rightPivot",      "rightPivot");
        deviceConf.put("leftExtension",   "leftExtension");
        deviceConf.put("rightExtension",  "rightExtension");
        deviceConf.put("wrist",           "pivot");
        deviceConf.put("reset",           "reset");

        Claw claw = new Claw(hardwareMap);
        Drive drive = new Drive(hardwareMap, deviceConf);

        Pivot pivot = new Pivot(hardwareMap, deviceConf);

        Extension extension = new Extension(hardwareMap, deviceConf);

        Wrist wrist = new Wrist(hardwareMap, deviceConf);

        Thread update = new Thread(()->updateAll());



        // Wait for the start button to be pressed
        waitForStart();
        claw.directSet(0.55);

        while (!pivot.checkReset() && opModeIsActive())
        {
            pivot.setDirectPos(pivot.getPos() - tickChange);
            telemetry.addData("pos", pivot.getPos());
            telemetry.addData("target", pivot.getPos() - 10);
            telemetry.addData("error", pivot.getError());
            telemetry.update();
            pivot.update();
            if (pivot.checkReset())
            {
                pivot.applyPower(0);
            }
        }
        pivot.setPos("High Specimen");
        pivot.setKP("High Specimen");
        extension.setPos("High Specimen");
        wrist.setPos("High Specimen");
        update.start();

        sleep(3000);

        drive.accelerateForward(u, v, t);
        telemetry.addData("slow", 1);
        telemetry.update();
        drive.applyPower(0);
        sleep(2000);
        pivot.setPos("High Spec Depo");
        sleep(5000);
        claw.directSet(clawOpen);
        extension.setPos("Idle");
        sleep(2000);
        pivot.setPos("Sample Intake");
        drive.accelerateForward(u, -v, t);
        drive.applyPower(0);

    }

    public void updateAll()
    {
        while (opModeIsActive())
        {

        pivot.setKP(extension.getPos());
        pivot.update();
        extension.update();
        }
    }

    public void sleep(int t)
    {
        try {
            Thread.sleep(t); // Wait for 1 millisecond
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
            // Optionally, log or handle the interruption
        }
    }

}