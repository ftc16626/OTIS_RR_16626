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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.CRServo;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedRight2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));
        //Wheel1 servo = hardwareMap.servo.get("Wheel1");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .lineToX(5)
                        .lineToX(0)
                        .build());


}

    public class Intake implements Action {

        double position;
        Servo servo;

        public Intake(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition()
            return false;
        }
    }
}