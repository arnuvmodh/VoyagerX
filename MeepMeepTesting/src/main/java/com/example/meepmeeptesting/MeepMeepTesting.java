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
                        .lineToSplineHeading(new Pose2d(startingX - 18.25, startingY + 6.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX - 9.8, startingY + 13.6583, 1.5808))
                        .lineToSplineHeading(new Pose2d(startingX - 18.25, startingY + 6.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX - 19, startingY + 13.9176, 1.5766))
                        .lineToSplineHeading(new Pose2d(startingX - 18.25, startingY + 6.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX - 22.5, startingY + 16.8352, 1.8539))
                        .lineToSplineHeading(new Pose2d(startingX - 18.25, startingY + 6.2, 0.75))
                        .splineToLinearHeading(new Pose2d(startingX + 17.5, startingY + 60, 6.2498), 0)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(startingX - 18.25, startingY + 6.2, 0.75), Math.toRadians(-90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}