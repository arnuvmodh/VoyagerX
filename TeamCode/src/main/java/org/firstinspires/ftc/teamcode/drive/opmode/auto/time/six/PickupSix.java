package org.firstinspires.ftc.teamcode.drive.opmode.auto.time.six;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

@Autonomous
public class PickupSix extends LinearOpMode {

    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        Trajectory targetTraj = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineToLinearHeading(new Pose2d(35, 23, Math.toRadians(90)), Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(targetTraj);
        while (opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
        }

    }

}