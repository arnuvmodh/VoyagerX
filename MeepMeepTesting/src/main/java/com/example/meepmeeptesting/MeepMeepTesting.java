package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        double startingX = 0;
        double startingY = 0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, 3.1121749877929688, 43.22031264858833, 15.28)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(startingX, startingY, 0))
                        // First Push (constant heading)
                        .lineToSplineHeading(new Pose2d(startingX - 30, startingY - 2, 0))
                        .splineTo(new Vector2d(startingX-20, startingY+26), Math.PI)
                        .splineToConstantHeading(new Vector2d(startingX - 55, startingY + 26), Math.PI)
                        .splineToConstantHeading(new Vector2d(startingX - 55, startingY + 35), Math.PI)
                        .splineToConstantHeading(new Vector2d(startingX - 15, startingY + 35), 0)

                        // Second Push (constant heading)
                        .splineToConstantHeading(new Vector2d(startingX - 55, startingY + 35), Math.PI)
                        .splineToConstantHeading(new Vector2d(startingX - 55, startingY + 45), Math.PI)
                        .splineToConstantHeading(new Vector2d(startingX - 15, startingY + 45), 0)

                        // Additional Movements
                                .splineToLinearHeading(new Pose2d(startingX - 5.8, startingY + 28.5, Math.PI), 0)
                                .splineToLinearHeading(new Pose2d(-28.3, -5, (2*Math.PI)-0.1), Math.PI)
                                .splineToLinearHeading(new Pose2d(startingX - 5.8, startingY + 28.5, Math.PI), 0)
                                .splineToLinearHeading(new Pose2d(startingX - 26.25, startingY - 12, 0), Math.PI)
                                .splineToLinearHeading(new Pose2d(startingX - 5.8, startingY + 28.5, Math.PI), 0)
                                .splineToLinearHeading(new Pose2d(startingX - 26.25, startingY - 12, 0), Math.PI)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}