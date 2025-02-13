package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        double startingX = -37;
        double startingY = -60;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, 3.1121749877929688, 43.22031264858833, 15.28)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(startingX, startingY, 0))
                        // Implementing trajectory sequences asynchronously
                        .lineToSplineHeading(new Pose2d(startingX-29.9025, startingY-3.7387, 0.0261))
                        .splineTo(new Vector2d(startingX-28.6438, startingY+18.4607), 2.0833)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}