

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import java.util.concurrent.TimeUnit;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;


@Disabled
@Autonomous(name="RedRightArm3", group="Shame")

public class RedRightWithOTOS extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         rotateArm = null;
    private DcMotor         extendArm = null;
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

        rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        Wheel1 = hardwareMap.get(CRServo.class, "Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        Wheel2 = hardwareMap.get(CRServo.class, "Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        rotateArm.setDirection(DcMotor.Direction.FORWARD);
        extendArm.setDirection(DcMotor.Direction.REVERSE);


        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          rotateArm.getCurrentPosition(),
                          extendArm.getCurrentPosition());
        telemetry.update();
        Pose2d startPose = new Pose2d(11.8, 61.7, Math.toRadians(90));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, startPose);
        //Trajectory t0 = drive.trajectoryBuilder(startPose)
              //  .splineTo(new Vector2d(-32.5,14.5), Math.toRadians(90)) //may need to change to linearheading
               // .build();
       // Trajectory t1 = drive.trajectoryBuilder(t0.end())
          /*    //  .strafeTo(new Vector2d(-60,15.5))
                .build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .lineTo(new Vector2d(-60,49))
                .build();

        // Wait for the game to start (driver presses START)
        waitForStart();
        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)






*/








                telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void encoderDrive(double speed, double armspeed,
                             double RotDegrees, double ExtInches, double wheel1Power, double wheel2Power,
                             double timeoutS) {

        int newROTarget;
        int newEXTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newROTarget = rotateArm.getCurrentPosition() + (int)(RotDegrees * COUNTS_PER_DEGREE);
            newEXTarget = extendArm.getCurrentPosition() + (int)(ExtInches * COUNTS_PER_EXTINCH);

            rotateArm.setTargetPosition(newROTarget);
            extendArm.setTargetPosition(newEXTarget);

            // Turn On RUN_TO_POSITION
            rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            rotateArm.setPower(Math.abs(armspeed));
            extendArm.setPower(Math.abs(speed));
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
                   (rotateArm.isBusy() || extendArm.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d");
                telemetry.addData("Currently at",  " at %7d :%7d",
                                              rotateArm.getCurrentPosition(), extendArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            rotateArm.setPower(.05);
            extendArm.setPower(0);
            Wheel1.setPower(0);
            Wheel2.setPower(0);

            // Turn off RUN_TO_POSITION

            rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        }
    }
}
