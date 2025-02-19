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
                        .lineToSplineHeading(new Pose2d(startingX-18.25, startingY+5.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX-14, startingY+10, 1.4318))
                        .lineToSplineHeading(new Pose2d(startingX-18.25, startingY+5.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX-21.25, startingY+10.27, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(startingX-18.25, startingY+5.2, 0.75))
                        .lineToSplineHeading(new Pose2d(startingX-22.2, startingY+13.47, 1.9))
                        .lineToSplineHeading(new Pose2d(startingX-18.25, startingY+5.2, 0.75))
                        .splineToLinearHeading(new Pose2d(startingX+17, startingX+64, 3.14159), 0)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}