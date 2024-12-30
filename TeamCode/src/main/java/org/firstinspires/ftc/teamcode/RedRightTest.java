package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "blueLeft", group = "Sensor")

public class RedRightTest extends LinearOpMode {

    public Servo Stick;
    public Servo ClawR; //Rotates Claw
    public CRServo ClawP;
    public CRServo ClawW;
    public DcMotor RAMotor;
    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode()
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");
        Stick = hardwareMap.servo.get("Stick");
        ClawR = hardwareMap.servo.get("ClawR"); //For rotation
        ClawR.setPosition(1);
        ClawP = hardwareMap.crservo.get("ClawP");//For wheels pixel
        ClawW = hardwareMap.crservo.get("ClawW");//For wheels pixel

        DcMotor RAMotor = hardwareMap.dcMotor.get("RAMotor");
        RAMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.update();

        Pose2d startPose = new Pose2d(-63.25,15.5);


        // Zone 1 actions
        Trajectory t0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-32.5,14.5), Math.toRadians(90)) //may need to change to linearheading
                .build();
        Trajectory t1 = drive.trajectoryBuilder(t0.end())
                .strafeTo(new Vector2d(-60,15.5))
                .build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .lineTo(new Vector2d(-60,49))
                .build();
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .strafeTo(new Vector2d(-42,49))
                .build();
        Trajectory t4 = drive.trajectoryBuilder(t3.end())
                .strafeTo(new Vector2d(-60,49))
                .build();
        // Zone 2 actions
        Trajectory t10 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-34.5,15.5))
                .build();
        Trajectory t11 = drive.trajectoryBuilder(t10.end())
                .lineToLinearHeading(new Pose2d(-36,49, Math.toRadians(90)))
                .build();
        Trajectory t12 = drive.trajectoryBuilder(t11.end())
                .strafeTo(new Vector2d(-60,49))
                .build();
        // Zone 3 actions
        Trajectory t5 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-32.5,15.5), Math.toRadians(-90)) //may need to change to linearheading
                .build();
        Trajectory t6 = drive.trajectoryBuilder(t5.end())
                .lineToLinearHeading(new Pose2d(-60,49, Math.toRadians(90)))
                .build();
        Trajectory t7 = drive.trajectoryBuilder(t6.end())
                .strafeTo(new Vector2d(-30,49))
                .build();
        Trajectory t8 = drive.trajectoryBuilder(t7.end())
                .strafeTo(new Vector2d(-60,49))
                .build();

        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            final int AREAONE = 105;
            final int AREATWO = 210;
            final int AREATHREE = 211;
            int zone = 0;

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */

            // The following 6 lines of code took like 8 days to get working

            Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Blocks to string", blocks.toString());
            for (int i = 0; i < blocks.length; i++) {
                int blockX = blocks[i].x;
                telemetry.addData("Block X", blockX);

                if (blockX <= AREAONE) {
                    zone = 1;
                } else if (blockX <= AREATWO){
                    zone = 2;
                } else if (blockX >= AREATHREE){
                    zone = 3;
                }

                telemetry.addData("Zone", zone);
            } /* code below here is added by derek, attempt/pseudo at using the roadrunner and having the bot
            move around as meant to
            */

            drive.setPoseEstimate(startPose);
            sleep(2000);
            if (zone == 1) {
                Stick.setPosition(0);
                sleep(100);
                drive.followTrajectory(t0);
                Stick.setPosition(.8);
                drive.followTrajectory(t1);
                RAMotor.setPower(1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(.5167); //Claw Rotation
                sleep(1000);//lift arm up for placement
                drive.followTrajectory(t2);
                ClawP.setPower(-1);
                ClawW.setPower(-1);
                sleep(1000);
                ClawP.setPower(0);
                ClawW.setPower(0);
                drive.followTrajectory(t3);
                drive.followTrajectory(t4);
                RAMotor.setPower(-1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(1);  //return arm position
                sleep(1000000000);

            }

            if (zone == 2) {
                Stick.setPosition(0);
                sleep(100);
                drive.followTrajectory(t10);
                Stick.setPosition(.8);
                drive.followTrajectory(t11);
                RAMotor.setPower(1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(.5167); //Claw Rotation
                sleep(1000);//lift arm up for placement
                ClawP.setPower(-1);
                ClawW.setPower(-1);
                sleep(1000);
                ClawP.setPower(0);
                ClawW.setPower(0);
                drive.followTrajectory(t12);
                RAMotor.setPower(-1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(1);  //return arm position
                sleep(1000000000);


            }
            if (zone == 3) {
                Stick.setPosition(0);
                sleep(100);
                drive.followTrajectory(t5);
                Stick.setPosition(.8);
                drive.followTrajectory(t6);
                RAMotor.setPower(1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(.5167); //Claw Rotation
                sleep(1000);//lift arm up for placement
                drive.followTrajectory(t7);
                ClawP.setPower(-1);
                ClawW.setPower(-1);
                sleep(1000);
                ClawP.setPower(0);
                ClawW.setPower(0);
                drive.followTrajectory(t8 );
                RAMotor.setPower(-1);
                sleep(950);
                RAMotor.setPower(0);
                ClawR.setPosition(1);  //return arm position
                sleep(1000000000);
            }

            telemetry.update();
        }
    }
}