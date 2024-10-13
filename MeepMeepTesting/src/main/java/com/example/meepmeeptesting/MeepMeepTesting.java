package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,19)
                .setConstraints(100, 60, 4 * PI, 2 * PI, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-32,-59, toRadians(-90)))


                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-15,-35.5),toRadians(90))
                                        .waitSeconds(1)
                                        .setReversed(false)
                                        .lineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                                        .waitSeconds(0.25)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                                        .waitSeconds(0.1)
                                        .lineToLinearHeading(new Pose2d(-24,-24,180))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}