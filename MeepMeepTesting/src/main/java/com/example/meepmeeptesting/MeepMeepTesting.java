package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, -65, Math.toRadians(90)))
                                .forward(36)
                                .back(3)
                                .waitSeconds(1)
                                .back(6)
                                .turn(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-64, -14),Math.toRadians(180))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-24, -11))
                                .back(48)
                                .turn(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(48,-38),Math.toRadians(0))
                                .forward(6)
                                .waitSeconds(1)
                                .back(6)
                                .strafeLeft(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
/*
.forward(24)
                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .turn(Math.toRadians(-90))
                                .build()
 */