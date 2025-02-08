package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "PIDAuto", group = "Autonomous")
public class PIDTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-72,8,0));
        CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        CRServo Wheel2 =  hardwareMap.crservo.get("Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        DcMotor rotateArm = hardwareMap.get(DcMotorEx.class, "rotateArm");
        DcMotor extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
        DcMotor extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");

        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setDirection(DcMotor.Direction.FORWARD);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();

        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-9,-64.125,Math.toRadians(90)))

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
    public class PIDArm implements Action {
        private PIDFController armcontroller;
        private PIDFController slidecontroller;

        public double p = 0, i = 0, d = 0;
        public double f = 0;

        public int target;
        final double COUNTS_PER_EXT_MOTOR_REV = 537.7;
        final double PULLEY_DIAMETER_INCHES = 1.5;// For figuring circumference
        final double COUNTS_PER_EXTINCH = (COUNTS_PER_EXT_MOTOR_REV) /
                (PULLEY_DIAMETER_INCHES * 3.1415);
        final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
        final double ROTATE_GEAR_REDUC = 2.0;
        final double COUNTS_PER_DEGREE = COUNTS_PER_ROT_MOTOR_REV / 180;
        DcMotorEx extendArm1;
        DcMotorEx extendArm2;
        DcMotorEx rotateArm;
        private boolean initialized = false;

        public PIDArm(DcMotorEx l, DcMotorEx r, DcMotorEx ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp, double stop) {
           this.extendArm1 = l;
           this.extendArm2 = r;
           this.rotateArm = ar;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                armcontroller = new PIDFController(p, i, d,f);
                slidecontroller = new PIDFController(p,i,d,f);
                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            }

            //controller.setPID(p, i, d);
            int armPos = rotateArm.getCurrentPosition();
            //double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / COUNTS_PER_DEGREE)) * f;

           // double power = pid * ff;

           // rotateArm.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();


            return false;
            }
    }

}
