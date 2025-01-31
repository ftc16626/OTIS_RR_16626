package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Birdy", group = "Autonomous")
public class RightSideLoserProgram extends LinearOpMode {
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
                drive.actionBuilder(new Pose2d(9,-64.125,Math.toRadians(90)))
                        .lineToY(-42)
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,6,76,1,0,0,2.5)) // ArmUP
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,9,37,1,0,0,2)) // ArmOut
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,2,0,1,0,0,.5)) // ArmOut
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-18.75,-100,-.85,0,0,1.8)) // ArmDown
                        .strafeToLinearHeading(new Vector2d(38,-43), Math.toRadians(81)) // Right
                        .strafeToLinearHeading(new Vector2d(38,-10), Math.toRadians(81)) // Forward
                        .strafeToLinearHeading(new Vector2d(45,-10), Math.toRadians(81)) // Right
                        .strafeToLinearHeading(new Vector2d(45,-56), Math.toRadians(81)) // Back
                        .strafeToLinearHeading(new Vector2d(63,-37), Math.toRadians(94)) // Forward
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,20,4,.85,-1,1,4)) // Grap Sample
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-10,0,-.85,0,0,1.8)) // ArmIn
                        .strafeToLinearHeading(new Vector2d(61,-60), Math.toRadians(81)) // Park








                        .build());

    }

    public class Intake implements Action {
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;
        ElapsedTime timer;
        double next;


        public Intake(CRServo ls, CRServo rs, double lp, double rp, double stop) {
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.next = stop;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
            }
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);
            return timer.seconds() < next;

        }
    }
    public class ArmOutRU implements Action {
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
        DcMotor rotateArm;
        double Rposition;
        double next;

        public ArmOutRU(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_EXT_MOTOR_REV = 537.7;
                final double PULLEY_DIAMETER_INCHES = 1.5;// For figuring circumference
                final double COUNTS_PER_EXTINCH = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos < end) {
                return timer.seconds() < next;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }
    }
    public class ArmInRD implements Action {
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
        DcMotor rotateArm;
        double Rposition;
        double next;

        public ArmInRD(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
            this.next = stop;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos > end) {
                return timer.seconds() < next;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }


    }

    public class ArmInRU implements Action {
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
        DcMotor rotateArm;
        double Rposition;

        public ArmInRU(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                rotateArm.setDirection(DcMotor.Direction.FORWARD);
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (timer.seconds() < 3) {
                return true;
            } else if (pos > end) {
                return true;
            } else {
                rotateArm.setDirection(DcMotor.Direction.REVERSE);
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }


        }


    }

    public class ArmOutRD implements Action {
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
        DcMotor rotateArm;
        double Rposition;

        public ArmOutRD(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                extendArm1.setDirection(DcMotor.Direction.REVERSE);
                extendArm2.setDirection(DcMotor.Direction.FORWARD);
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos > end) {
                return true;
            } else {
                extendArm1.setDirection(DcMotor.Direction.FORWARD);
                extendArm2.setDirection(DcMotor.Direction.REVERSE);
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }


    }

}
