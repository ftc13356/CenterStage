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
                                drive.trajectorySequenceBuilder(new Pose2d(-55,-35, toRadians(180)))


                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-40,-58.5),toRadians(0))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32,5,15))
                                        .splineToConstantHeading(new Vector2d(-35,-58.5),toRadians(0))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                                        .splineToConstantHeading(new Vector2d(5,-58.5),toRadians(0))
//                                        .splineToConstantHeading(new Vector2d(40,-36),toRadians(0))
                                        .splineToConstantHeading(new Vector2d(48,-36),toRadians(0))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}