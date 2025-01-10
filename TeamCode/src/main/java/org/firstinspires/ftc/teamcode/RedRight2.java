package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
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


@Autonomous(name = "DAMN", group = "Autonomous")
public class RedRight2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-72,8,0));
        CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        CRServo Wheel2 =  hardwareMap.crservo.get("Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        DcMotor rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        DcMotor extendArm1 = hardwareMap.get(DcMotor.class, "extendArm1");
        DcMotor extendArm2 = hardwareMap.get(DcMotor.class, "extendArm2");

        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setDirection(DcMotor.Direction.FORWARD);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-9,-64.125,Math.toRadians(90)))
                        .lineToY(-37.9)
                        .stopAndAdd(new RotateUp(rotateArm, 110))
                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, Wheel1, Wheel2, 13.5, 1, 0, 0))
                        .stopAndAdd(new RotateDown(rotateArm, -8))
                        .stopAndAdd(new ExtendIn(extendArm1, extendArm2, Wheel1, Wheel2, -12, -1, -1,1))
                        .stopAndAdd(new RotateDown(rotateArm, -84))
                        .strafeToLinearHeading(new Vector2d( -46, -47 ), Math.toRadians(88))
                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, Wheel1, Wheel2, 14, .7,-1,1))
                        .build());

    }

    public class ExtendOut implements Action {
        DcMotor extendArm1;
        DcMotor extendArm2;
        double Aposition;
        private boolean initialized = false;
        ElapsedTime timer;
        double speed;
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;

        public ExtendOut(DcMotor l, DcMotor r, CRServo ls, CRServo rs, double ext, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition();
            double end = extendArm2.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos < end) {
                return true;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }


    }
    public class ExtendIn implements Action {
        DcMotor extendArm1;
        DcMotor extendArm2;
        double Aposition;
        private boolean initialized = false;
        ElapsedTime timer;
        double speed;
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;

        public ExtendIn(DcMotor l, DcMotor r, CRServo ls, CRServo rs, double ext, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition();
            double end = extendArm2.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos > end) {
                return true;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }


    }

    public class RotateUp implements Action {
        DcMotor rotateArm;
        double Rposition;
        private boolean initialized = false;
        ElapsedTime timer;

        public RotateUp(DcMotor l, double rot) {
            this.rotateArm = l;
            this.Rposition = rot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;

                int newROTarget;
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(Math.abs(1));
                initialized = true;
            }

            double pos = rotateArm.getCurrentPosition();
            double end = rotateArm.getTargetPosition();
            if (pos < end) {
                return true;
            } else {
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }



        }
    }
    public class RotateDown implements Action {
        DcMotor rotateArm;
        double Rposition;
        private boolean initialized = false;
        ElapsedTime timer;

        public RotateDown(DcMotor l, double rot) {
            this.rotateArm = l;
            this.Rposition = rot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;

                int newROTarget;
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = rotateArm.getCurrentPosition();
            double end = rotateArm.getTargetPosition();
            if (pos > end) {
                return true;
            } else {
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }



        }
    }
}
