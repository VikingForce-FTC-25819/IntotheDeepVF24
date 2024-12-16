package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DriveConstants.MAX_ACCEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_VEL;
import static com.example.meepmeeptesting.DriveConstants.TRACK_WIDTH;

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
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8, -42, Math.toRadians(90)))
//                                .setReversed(true)
                                .splineTo(new Vector2d(46, -15), Math.toRadians(270))
//                                .splineToSplineHeading(new Pose2d(36, -9, Math.toRadians(270)), Math.toRadians(125)).setReversed(true)
//                                .splineToLinearHeading(new Pose2d(36, -34, Math.toRadians(270)), Math.toRadians(115))
//                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(36, -42, Math.toRadians(270)))
                                //.splineToLinearHeading(new Pose2d(46, -9, Math.toRadians(270)), Math.toRadians(270))
                                //.splineToConstantHeading()
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Adambots2\\vf\\field-2024-official.png")); }
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
