package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, -65, Math.toRadians(90)))
                                .forward(30)
                                .back(8)
                                .strafeLeft(36)
                                .forward(8)
                                .turn(Math.toRadians(160))
                                .forward(22)
                                .back(16)
                                .turn(Math.toRadians(110))
                                .forward(60)
                                .turn(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(48,-51),Math.toRadians(0))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Adambots 1\\Documents\\field-2024-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .addEntity(myBot)
                .start();

        /*meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start(); */
    }
}
