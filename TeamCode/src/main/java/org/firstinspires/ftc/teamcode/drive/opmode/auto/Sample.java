package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

@Autonomous()
public class Sample extends LinearOpMode {
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 1;
    double horizontalSlidePosition = 0;

    State curState = State.idle;

    enum State {
        scorePreload,
        grabFirst,
        scoreFirst,
        grabSecond,
        scoreSecond,
        grabThird,
        scoreThird,
        park,
        idle
    }

    Robot robot;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.intakePivot.flipBack();
        robot.outtakePivot.flipFront();
        robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.55);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        Trajectory scorePreload = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-23, 4, 0.9))
                .build();
        Trajectory grabFirst = drive.trajectoryBuilder(scorePreload.end())
                .lineToSplineHeading(new Pose2d(-14, 10, 1.4318))
                .build();
        Trajectory scoreFirst = drive.trajectoryBuilder(grabFirst.end())
                .lineToSplineHeading(new Pose2d(-24, 5, 0.9))
                .build();
        Trajectory grabSecond = drive.trajectoryBuilder(scoreFirst.end())
                .lineToSplineHeading(new Pose2d(-18, 10, 1.6415))
                .build();
        Trajectory scoreSecond = drive.trajectoryBuilder(grabSecond.end())
                .lineToSplineHeading(new Pose2d(-24, 5, 0.9))
                .build();
        Trajectory grabThird = drive.trajectoryBuilder(scoreSecond.end())
                .lineToSplineHeading(new Pose2d(-22.2, 13.47, 1.9))
                .build();
        Trajectory scoreThird = drive.trajectoryBuilder(grabThird.end())
                .lineToSplineHeading(new Pose2d(-24, 5, 0.9))
                .build();
        Trajectory park = drive.trajectoryBuilder(scoreThird.end())
                .lineToSplineHeading(new Pose2d(-13.6212, 7.5, (Math.PI / 2) + 0.2))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(scorePreload);
        curState = State.scorePreload;

        while (opModeIsActive()) {
            switch (curState) {
                case scorePreload:
                    if (!drive.isBusy()) {
                        robot.outtakePivot.flipTo(0.65); // Flip to scoring position
                        if (robot.outtakePivot.isAt(0.65)) {
                            drive.followTrajectoryAsync(grabFirst);
                            curState = State.grabFirst;
                        }
                    }
                    break;

                case grabFirst:
                    if (!drive.isBusy()) {
                        robot.intakePivot.flipFront();
                        if (robot.intakePivot.isAt(0.8)) {
                            robot.intakeClaw.close();
                            if (robot.intakeClaw.isClosed()) {
                                drive.followTrajectoryAsync(scoreFirst);
                                curState = State.scoreFirst;
                            }
                        }
                    }
                    break;

                case scoreFirst:
                    if (!drive.isBusy()) {
                        robot.outtakePivot.flipTo(0.65);
                        if (robot.outtakePivot.isAt(0.65)) {
                            robot.outtakeClaw.open();
                            if (robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) {
                                drive.followTrajectoryAsync(grabSecond);
                                curState = State.grabSecond;
                            }
                        }
                    }
                    break;

                case grabSecond:
                    if (!drive.isBusy()) {
                        robot.intakePivot.flipFront();
                        if (robot.intakePivot.isAt(0.8)) {
                            robot.intakeClaw.close();
                            if (robot.intakeClaw.isClosed()) {
                                drive.followTrajectoryAsync(scoreSecond);
                                curState = State.scoreSecond;
                            }
                        }
                    }
                    break;

                case scoreSecond:
                    if (!drive.isBusy()) {
                        robot.outtakePivot.flipTo(0.65);
                        if (robot.outtakePivot.isAt(0.65)) {
                            robot.outtakeClaw.open();
                            if (robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) {
                                drive.followTrajectoryAsync(grabThird);
                                curState = State.grabThird;
                            }
                        }
                    }
                    break;

                case grabThird:
                    if (!drive.isBusy()) {
                        robot.intakePivot.flipFront();
                        if (robot.intakePivot.isAt(0.8)) {
                            robot.intakeClaw.close();
                            if (robot.intakeClaw.isClosed()) {
                                drive.followTrajectoryAsync(scoreThird);
                                curState = State.scoreThird;
                            }
                        }
                    }
                    break;

                case scoreThird:
                    if (!drive.isBusy()) {
                        robot.outtakePivot.flipTo(0.65);
                        if (robot.outtakePivot.isAt(0.65)) {
                            robot.outtakeClaw.open();
                            if (robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) {
                                drive.followTrajectoryAsync(park);
                                curState = State.park;
                            }
                        }
                    }
                    break;

                case park:
                    if (!drive.isBusy()) {
                        curState = State.idle;
                    }
                    break;

                case idle:
                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("Current State", curState);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}