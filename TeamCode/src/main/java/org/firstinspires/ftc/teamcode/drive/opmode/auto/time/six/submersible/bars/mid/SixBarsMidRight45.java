package org.firstinspires.ftc.teamcode.drive.opmode.auto.time.six.submersible.bars.mid;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.time.six.SixSample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bars 6 Mid R 45°", group = "Bars 6")
public class SixBarsMidRight45 extends SixSample {
    @Override
    protected Pose2d getSampleSixTrajectory() {
        return new Pose2d(24.6164, 25.3708, 1);
    }

    @Override
    protected double getIntakeSlidePosition() {
        return 0.675;
    }

    @Override
    protected double getIntakeClawPivotPosition() {
        return 0.5;
    }
}