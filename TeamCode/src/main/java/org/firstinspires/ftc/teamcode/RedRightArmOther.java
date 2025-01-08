

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="RedLeftSample", group="SecondBot")

public class RedRightArmOther extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LFMotor   = null;
    private DcMotor         RFMotor  = null;
    private DcMotor         LBMotor   = null;
    private DcMotor         RBMotor  = null;
    private DcMotor         rotateArm = null;
    private DcMotor         extendArm = null;
    private DcMotor         extendArm2 = null;
    CRServo Wheel1;
    CRServo Wheel2;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_DRIVE_MOTOR_REV   = 384.5;
    static final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
    static final double     COUNTS_PER_ROT_MOTOR_REV    = 1993.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_DRIVE_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);
    static final double     ROTATE_GEAR_REDUC = 2.0 ;
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;
    static final double     ROT_SPEED               = 1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm1");
        extendArm2 = hardwareMap.get(DcMotor.class, "extendArm2");
        Wheel1 = hardwareMap.get(CRServo.class, "Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        Wheel2 = hardwareMap.get(CRServo.class, "Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm.setDirection(DcMotor.Direction.FORWARD);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                LFMotor.getCurrentPosition(),
                LBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                rotateArm.getCurrentPosition(),
                extendArm.getCurrentPosition(),
                extendArm2.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        sleep(3000);
        encoderDrive(0.7, ROT_SPEED,-5, -5, -5, -5, true, 0, 0 , 0, 0, 5); // Go to specimen rack, rotate & arm
        encoderDrive(0.7, ROT_SPEED,24, 24, 24, 24, false, 133, 10 , 0, 0, 5); // Go to specimen rack, rotate & arm
        encoderDrive(0.7, .6,0, 0, 0, 0, false, 0, 6.5, 0, 0, 5); // Finish extension
        encoderDrive(0.7, ROT_SPEED, 0, 0, 0 , 0, false, -63, -15.5, -1, 1, 5); // Hook Specimen
        encoderDrive(0.7, ROT_SPEED, -9, -9, -9 , -9, false, -29, 0 , 0, 0, 5); // Reverse
        encoderDrive(0.7, ROT_SPEED, 50, 50, 50 , 50, true, -10, 0 , 0, 0, 5); // Strafe Left Sample 1
        encoderDrive(.5, ROT_SPEED, 0, 0, 0 , 0, false, 0, 16 , -1, 1, 5); // Intake Sample
        encoderDrive(.5, ROT_SPEED, 0, 0, 0 , 0, false, 35, -5 , 0, 0, 1); // Over Sec Sample
        encoderDrive(0.7, ROT_SPEED, -33, -33, 33, 33, false, 110, 0 , 0, 0, 3); // TurntoBucket
        encoderDrive(0.7, ROT_SPEED, 17, 17, 17, 17, false, 10, 14 , 0, 0, 3); // Forward Extend
        encoderDrive(0.7, ROT_SPEED, 0, 0, 0 , 0, false, 0, 0 , 1, -1, 1); // Outtake
        encoderDrive(0.7, ROT_SPEED, 0, 0, 0 , 0, false, 0, -25 , 0, 0, 3); // Tuck
        encoderDrive(0.7, ROT_SPEED, 45, 45, -45 , -45, false, -100, 0 , 0, 0, 2); // Turn
        encoderDrive(0.7, ROT_SPEED, 70, 70, 50 , 50, false, 40, 15 , 0, 0, 3); // Park


        // encoderDrive(0.7, ROT_SPEED, 38, 38, -38 , -38, false, -80, 0 , 0, 0, 2); // Turn
        //encoderDrive(0.7, ROT_SPEED, 13, 13, 13 , 13, false, 0, 5 , -1, 1, 5); // Forward n Grab
       // encoderDrive(0.7, ROT_SPEED, -50, -50, 50 , 50, false, 100, -5 , 0, 0, 5); // 180 Turn
       // encoderDrive(0.7, ROT_SPEED, 13, 13, 15 , 15, false, 10, 18 , 0, 0, 5); // Forward Extend
       // encoderDrive(0.7, ROT_SPEED, 0, 0, 0 , 0, false, 0, 0 , 1, -1, 5); // Outtake
       // encoderDrive(0.7, ROT_SPEED, 30, 30, -30 , -30, false, -10, -18 , 0, 0, 5); // Turn
       // encoderDrive(0.7, ROT_SPEED, -40, -40, -40 , -40, true, -40, 0 , 0, 0, 5); // Park Strafe














        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void encoderDrive(double speed, double armspeed,
                             double leftFInches, double leftBInches,
                             double rightFInches, double rightBInches,
                             boolean Strafe,
                             double RotDegrees, double ExtInches, double wheel1Power, double wheel2Power,
                             double timeoutS) {
        int newLFTarget;
        int newLBTarget;
        int newRFTarget;
        int newRBTarget;
        int newROTarget;
        int newEXTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            if (Strafe) {
                LFMotor.setDirection(DcMotor.Direction.FORWARD);
                RBMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                LFMotor.setDirection(DcMotor.Direction.REVERSE);
                RBMotor.setDirection(DcMotor.Direction.FORWARD);
            }




            // Determine new target position, and pass to motor controller
            newLFTarget = LFMotor.getCurrentPosition() + (int)(leftFInches * COUNTS_PER_INCH);
            newLBTarget = LBMotor.getCurrentPosition() + (int)(leftBInches * COUNTS_PER_INCH);
            newRFTarget = RFMotor.getCurrentPosition() + (int)(rightFInches * COUNTS_PER_INCH);
            newRBTarget = RBMotor.getCurrentPosition() + (int)(rightBInches * COUNTS_PER_INCH);
            newROTarget = rotateArm.getCurrentPosition() + (int)(RotDegrees * COUNTS_PER_DEGREE);
            newEXTarget = extendArm.getCurrentPosition() + (int)(ExtInches * COUNTS_PER_EXTINCH);

            LFMotor.setTargetPosition(newLFTarget);
            RFMotor.setTargetPosition(newRFTarget);
            LBMotor.setTargetPosition(newLBTarget);
            RBMotor.setTargetPosition(newRBTarget);
            rotateArm.setTargetPosition(newROTarget);
            extendArm.setTargetPosition(newEXTarget);
            extendArm2.setTargetPosition(newEXTarget);

            // Turn On RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            LFMotor.setPower(Math.abs(speed));
            RFMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));
            rotateArm.setPower(1 * Math.abs(armspeed));
            extendArm.setPower(Math.abs(speed));
            extendArm2.setPower(1 * Math.abs(speed));
            Wheel1.setPower(wheel1Power);
            Wheel2.setPower(wheel2Power);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            //onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFMotor.isBusy() || rotateArm.isBusy() || extendArm.isBusy() ||extendArm2.isBusy() || RFMotor.isBusy() || LBMotor.isBusy() || RBMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLFTarget, newRFTarget,  newRBTarget,  newLBTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition(), rotateArm.getCurrentPosition(), extendArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);
            rotateArm.setPower(.05);
            extendArm.setPower(0);
            extendArm2.setPower(0);
            Wheel1.setPower(0);
            Wheel2.setPower(0);

            // Turn off RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
