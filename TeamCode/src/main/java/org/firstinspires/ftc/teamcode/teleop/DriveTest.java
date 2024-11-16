package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class DriveTest extends LinearOpMode {
    public static double test1 = 0.6, test2 = 1;
    public static int  test3 = 100;
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor leftPivotMotor = hardwareMap.dcMotor.get("leftPivot");
        DcMotor rightPivotMotor = hardwareMap.dcMotor.get("rightPivot");

        DcMotor leftExtension = hardwareMap.dcMotor.get("leftExtension");
        DcMotor rightExtension = hardwareMap.dcMotor.get("rightExtension");

        Servo pivot = hardwareMap.servo.get("pivot");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //comment
        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a) {
                leftPivotMotor.setTargetPosition(leftPivotMotor.getCurrentPosition()+5);
                rightPivotMotor.setTargetPosition(rightPivotMotor.getCurrentPosition()+5);
                leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftPivotMotor.setPower(1);
                rightPivotMotor.setPower(1);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                leftPivotMotor.setTargetPosition(leftPivotMotor.getCurrentPosition()-5);
                rightPivotMotor.setTargetPosition(rightPivotMotor.getCurrentPosition()-5);
                leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftPivotMotor.setPower(1);
                rightPivotMotor.setPower(1);
            }

            if (gamepad2.a)
            {
                leftPivotMotor.setTargetPosition(0);
                rightPivotMotor.setTargetPosition(0);
                leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftPivotMotor.setPower(1);
                rightPivotMotor.setPower(1);
                // pivot.setPosition(0.1);
            }
            if (gamepad2.b)
            {
                leftPivotMotor.setTargetPosition(-325);
                rightPivotMotor.setTargetPosition(-325);
                leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftPivotMotor.setPower(1);
                rightPivotMotor.setPower(1);
                //pivot.setPosition(0.5);
            }

            if (gamepad1.y)
            {
                pivot.setPosition(0);


            }
            if (gamepad1.x)
            {
                pivot.setPosition(1);

            }

            if (gamepad2.dpad_up)
            {
                leftExtension.setTargetPosition(3425);
                rightExtension.setTargetPosition(-3425);
                leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftExtension.setPower(1);
                rightExtension.setPower(1);
            }
            if (gamepad2.dpad_down)
            {
                leftExtension.setTargetPosition(0);
                rightExtension.setTargetPosition(0);
                leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftExtension.setPower(1);
                rightExtension.setPower(1);
            }

            telemetry.addData("tick", leftPivotMotor.getCurrentPosition());
            telemetry.addData("r", rightPivotMotor.getCurrentPosition());
            telemetry.addData("tick", leftExtension.getCurrentPosition());
            telemetry.addData("r", rightExtension.getCurrentPosition());
            telemetry.addData("pivot", pivot.getPosition());

            telemetry.update();


        }
    }
}