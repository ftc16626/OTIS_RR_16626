
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class TetsyWetsyUwu extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        int target=100;
        double extendo=0;
        int clawgrab=1;
        int clawroll=1;
        double clawpitch=0.5;
        int btarget=0;
        boolean pressing=false;
        boolean pressing2=false;
        Servo linkage1 = hardwareMap.servo.get("linkage1");
        Servo linkage2 = hardwareMap.servo.get("linkage2");
        Servo clawfingers = hardwareMap.servo.get("clawfingers");
        Servo clawturn = hardwareMap.servo.get("clawturn");
        Servo clawwrist = hardwareMap.servo.get("clawwrist");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class,"arm");
        DcMotorEx bucket = hardwareMap.get(DcMotorEx.class,"bucket");
        linkage1.setDirection(Servo.Direction.REVERSE);
        linkage1.setPosition(0);
        linkage2.setPosition(0);
        waitForStart();
        if (isStopRequested()) {return;}
        while (opModeIsActive()) {
            if (gamepad1.left_bumper){
                target-=15;
            }
            else if (gamepad1.right_bumper){
                target+=15;
            }
            if (gamepad1.left_trigger>0){
                extendo-=15;
            }
            else if (gamepad1.right_trigger>0){
                extendo+=15;
            }
            if (gamepad1.a){
                if (!pressing){
                    clawgrab=Math.abs(1-clawgrab);
                }
                pressing=true;
            }
            else{
                pressing=false;
            }
            if (gamepad1.b){
                if (!pressing2){
                    clawroll=Math.abs(1-clawroll);
                }
                pressing2=true;
            }
            else{
                pressing2=false;
            }
            if (gamepad1.left_stick_y>0.2){
                btarget+=15;
            }
            else if (gamepad1.left_stick_y<-0.2){
                btarget-=15;
            }
            if (gamepad2.right_stick_y<-0.3) {
                clawpitch = 0;
            }
            else if (gamepad2.right_stick_y>-0.3 && gamepad2.right_stick_y<0.3){
                clawpitch = 0.5;
            }
            else{
                clawpitch = 1;
            }

            int error1 = (target - arm.getCurrentPosition());
            arm.setPower(-error1 * 0.015);
            int error2 = (btarget - bucket.getCurrentPosition());
            bucket.setPower(-error2 * 0.015);
            linkage1.setPosition(extendo);
            linkage1.setPosition(extendo);
            clawfingers.setPosition(clawgrab);
            clawturn.setPosition(clawroll);
            clawwrist.setPosition(clawpitch);
            telemetry.addData("armtarget", target);
            telemetry.addData("armpos", arm.getCurrentPosition());
            telemetry.addData("buckettarget", btarget);
            telemetry.addData("bucketpos", bucket.getCurrentPosition());
            telemetry.addData("extendo", extendo);
            telemetry.addData("clawgrab", clawgrab);
            telemetry.addData("clawroll", clawroll);
            telemetry.addData("clawpitch", clawpitch);
            telemetry.update();
        }

    }
}
