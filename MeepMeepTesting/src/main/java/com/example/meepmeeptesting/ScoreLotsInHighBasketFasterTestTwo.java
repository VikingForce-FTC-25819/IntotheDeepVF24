package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DriveConstants.MAX_ACCEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_VEL;
import static com.example.meepmeeptesting.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import javax.imageio.ImageIO;

public class ScoreLotsInHighBasketFasterTestTwo {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d start = new Pose2d(-16, -63, Math.toRadians(90));

        Trajectory moveOffWall = trajectoryBuilder(start)
                .forward(2)
                .build();

        Trajectory trajectoryScore = trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeThree = trajectoryBuilder(trajectoryScore.end())
                .lineToLinearHeading(new Pose2d(-34, -48, Math.toRadians(135)))
                .build();

        Trajectory pickUpSpikeThree = trajectoryBuilder(prepareForPickUpSpikeThree.end())
                //.lineToLinearHeading(new Pose2d(-57, -44, Math.toRadians(90)))
                .forward(5,
                        getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeThree = trajectoryBuilder(pickUpSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeTwo = trajectoryBuilder(trajectoryScoreFromSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-40, -35, Math.toRadians(155)))
                .build();

        Trajectory pickUpSpikeTwo = trajectoryBuilder(prepareForPickUpSpikeTwo.end())
                .forward(6,
                        getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeTwo = trajectoryBuilder(pickUpSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeOne = trajectoryBuilder(trajectoryScoreFromSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-43, -43, Math.toRadians(150)))
                .build();

        Trajectory pickUpSpikeOne = trajectoryBuilder(prepareForPickUpSpikeOne.end())
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeOne = trajectoryBuilder(pickUpSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory rotateForPark = trajectoryBuilder(trajectoryScoreFromSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(90)))
                .build();

        Trajectory parkInAscentZone = trajectoryBuilder(rotateForPark.end())
                .splineTo(new Vector2d(-24, -9), Math.toRadians(0))
                .build();

        RoadRunnerBotEntity mybot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, TRACK_WIDTH)
                .setStartPose(start)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .addTrajectory(moveOffWall)
                                .addTrajectory(trajectoryScore)
                                .addTrajectory(prepareForPickUpSpikeThree)
                                .addTrajectory(pickUpSpikeThree)
                                .addTrajectory(trajectoryScoreFromSpikeThree)
                                .addTrajectory(prepareForPickUpSpikeTwo)
                                .addTrajectory(pickUpSpikeTwo)
                                .addTrajectory(trajectoryScoreFromSpikeTwo)
                                .addTrajectory(prepareForPickUpSpikeOne)
                                .addTrajectory(pickUpSpikeOne)
                                .addTrajectory(trajectoryScoreFromSpikeOne)
                                .addTrajectory(rotateForPark)
                                .addTrajectory(parkInAscentZone)
                                .build()
                );


        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\Adambots2\\vf\\field-2024-official.png"));
        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .addEntity(mybot)
                .start();
    }

    private static TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
