package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 16)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(42, 64, 3*Math.PI/2))
                        .lineToY(60)
                        .turnTo(-0.2783)
                .strafeTo(new Vector2d(56, 56))
                        .turnTo(Math.toRadians(225))

                        .turnTo(-1.95)
                .strafeTo(new Vector2d(48, 36))
                        .turnTo(Math.toRadians((270)))

                .turnTo(-1.95)
                .strafeTo(new Vector2d(56, 56))
                .turnTo(Math.toRadians(225))

                .turnTo(-1.471)
                .strafeTo(new Vector2d(58, 36))
                .turnTo(Math.toRadians((270)))

                .turnTo(-1.471)
                .strafeTo(new Vector2d(56, 56))
                .turnTo(Math.toRadians(225))

                .turnTo(-1.234)
                .strafeTo(new Vector2d(63, 36))

                .strafeTo(new Vector2d(56, 56))
                .turnTo(Math.toRadians(225))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}