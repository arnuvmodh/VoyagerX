package org.firstinspires.ftc.teamcode.drive.opmode.auto.time.five.submersible.ascent.mid;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.time.five.FiveSample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Ascent 5 Mid 0°", group = "Bars 5")
public class FiveAscentMid0 extends FiveSample {
    @Override
    protected Pose2d getSampleFiveTrajectory() {
        return new Pose2d(13, 62.3794, 6.2498);
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