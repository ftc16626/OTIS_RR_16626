package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.PI, Math.PI, 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        .forward(25)
                        .back(4)
                        .strafeTo(new Vector2d(-48,-38))
                        .turn(Math.toRadians(145))
                        .forward(15)
                        .turn(Math.toRadians(-145))
                        .strafeLeft(2)
                        .forward(12)
                        .back(12)
                        .turn(Math.toRadians(145))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}