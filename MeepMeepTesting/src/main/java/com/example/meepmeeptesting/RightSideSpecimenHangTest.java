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

public class RightSideSpecimenHangTest {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d start = new Pose2d(16, -63, Math.toRadians(90));

        Trajectory moveOffWall = trajectoryBuilder(start)
                .forward(5)
                .build();

        Trajectory alignToScore = trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(8, -58, Math.toRadians(90)))
                .build();

        Trajectory trajectoryScore = trajectoryBuilder(alignToScore.end())
                .forward(22)
                .build();

        Trajectory lockItIn = trajectoryBuilder(trajectoryScore.end())
                .forward(2,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromHang = trajectoryBuilder(lockItIn.end())
                .back(5)
                .build();

        Trajectory strafeRightClearOfSubmersible = trajectoryBuilder(backUpFromHang.end())
                .strafeRight(26)
                .build();

        Trajectory forwardOfSpikeMarks = trajectoryBuilder(strafeRightClearOfSubmersible.end())
                .forward(30)
                .build();

        Trajectory strafeToSpikeThree = trajectoryBuilder(forwardOfSpikeMarks.end())
                .strafeRight(13)
                .build();

        Trajectory strafeToSpikeTwo = trajectoryBuilder(forwardOfSpikeMarks.end())
                .strafeRight(22)
                .build();

        Trajectory pushToObservationZoneSpikeThree = trajectoryBuilder(strafeToSpikeThree.end())
                .back(48)
                .build();


        Trajectory pushToObservationZoneSpikeTwo = trajectoryBuilder(strafeToSpikeTwo.end())
                .back(48)
                .build();

        Trajectory lineUpForSpecimenPickUp = trajectoryBuilder(pushToObservationZoneSpikeThree.end())
                .lineToLinearHeading(new Pose2d(48, -43, Math.toRadians(270)))
                .build();

        Trajectory pickUpSpecimenOne = trajectoryBuilder(lineUpForSpecimenPickUp.end())
                .forward(8)
                .build();

        Trajectory alignToScoreFromFirstPickUp = trajectoryBuilder(pickUpSpecimenOne.end())
                .lineToLinearHeading(new Pose2d(6, -58, Math.toRadians(90)))
                .build();

        Trajectory parkInObservation = trajectoryBuilder(pushToObservationZoneSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(63, -60, Math.toRadians(90)))
                .build();

        RoadRunnerBotEntity mybot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL, TRACK_WIDTH)
                .setStartPose(start)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .addTrajectory(moveOffWall)
                                .addTrajectory(trajectoryScore)
                                .addTrajectory(lockItIn)
                                .addTrajectory(backUpFromHang)
                                .addTrajectory(strafeRightClearOfSubmersible)
                                .addTrajectory(forwardOfSpikeMarks)
                                .addTrajectory(strafeToSpikeThree)
                                .addTrajectory(pushToObservationZoneSpikeThree)
                                .addTrajectory(lineUpForSpecimenPickUp)
                                .addTrajectory(pickUpSpecimenOne)
                                .addTrajectory(alignToScoreFromFirstPickUp)
                                .addTrajectory(trajectoryScore)
                                .addTrajectory(lockItIn)
                                .addTrajectory(backUpFromHang)
                                .addTrajectory(strafeRightClearOfSubmersible)
                                .addTrajectory(forwardOfSpikeMarks)
                                .addTrajectory(strafeToSpikeTwo)
                                .addTrajectory(pushToObservationZoneSpikeTwo)
                                .addTrajectory(parkInObservation)

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
