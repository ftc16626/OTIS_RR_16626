package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedRight2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));
        CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        CRServo Wheel2 =  hardwareMap.crservo.get("Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        DcMotor rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        DcMotor extendArm1 = hardwareMap.get(DcMotor.class, "extendArm1");
        DcMotor extendArm2 = hardwareMap.get(DcMotor.class, "extendArm2");

        rotateArm.setDirection(DcMotor.Direction.FORWARD);
        extendArm1.setDirection(DcMotor.Direction.REVERSE);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-63.5,0,0))
                        .splineTo(new Vector2d(0, -10), Math.PI / 2)
                        .build());


    }

    public class Intake implements Action {
        Servo Wheel1;
        double position;


        public Intake(Servo s, double p) {
            this.Wheel1 = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Wheel1.setPosition(position);
            return false;
        }
    }

    public class RedRightArm implements Action {
        DcMotor extendArm1;
        DcMotor extendArm2;
        DcMotor rotateArm;
        double RotDegrees;
        double ExtInches;
        double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
        double     COUNTS_PER_ROT_MOTOR_REV    = 1993.6;
        double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
        double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                (PULLEY_DIAMETER_INCHES * 3.1415);
        double     ROTATE_GEAR_REDUC = 2.0 ;
        double     COUNTS_PER_DEGREE       = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
        double     EXT_SPEED             = 1;
        double     ROT_SPEED               = 1;
        double     runtime;
        ElapsedTime timer;
        boolean hasinitialized;
        int newROTarget;
        int newEXTarget1;
        int newEXTarget2;


        public RedRightArm(DcMotor l, DcMotor r, DcMotor rot, double rs, double s, double h, double p, double t) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.rotateArm = rot;
            this.EXT_SPEED = s;
            this.ROT_SPEED = rs;
            this.ExtInches = h;
            this.RotDegrees = p;
            this.runtime = t;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(hasinitialized) {

                newROTarget = rotateArm.getCurrentPosition() + (int)(RotDegrees * COUNTS_PER_DEGREE);
                newEXTarget1 = extendArm1.getCurrentPosition() + (int)(ExtInches * COUNTS_PER_EXTINCH);
                newEXTarget2 = extendArm2.getCurrentPosition() + (int)(ExtInches * COUNTS_PER_EXTINCH);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newEXTarget1);
                extendArm2.setTargetPosition(newEXTarget2);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                timer.reset();
                rotateArm.setPower(Math.abs(ROT_SPEED));
                extendArm1.setPower(Math.abs(EXT_SPEED));






            }
            return (timer.seconds() <  runtime) &&
                    ((rotateArm.isBusy() || extendArm1.isBusy() ));
    }


}}