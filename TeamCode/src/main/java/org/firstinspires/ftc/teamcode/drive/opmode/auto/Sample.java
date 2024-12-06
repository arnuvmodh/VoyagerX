package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class Sample extends LinearOpMode {
    private Robot robot;
    final double SPECIMEN_GRAB_POSITION = 0.2;
    final double SPECIMEN_SCORE_POSITION = 0.65;
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 1;
    final double INTAKE_CLAW_CLOSE_POSITION = 0.55;
    private SampleMecanumDrive drive;
    public Trajectory preloadOuttake, firstIntake, firstOuttake, secondIntake, secondOuttake, thirdIntake, thirdOuttake, park;
    enum State {
        idle,
        preloadOuttake,
        firstIntake,
        firstOuttake,
        secondIntake,
        secondOuttake,
        thirdIntake,
        thirdOuttake,
        park
    }
    private final Pose2d highBucketPosition = new Pose2d(-18.25, 5.2, 0.75);
    private final Vector2d firstSamplePosition = new Vector2d(-19.25, 5.5);
    private final Vector2d secondSamplePosition = new Vector2d(-20.75, 9.55);
    private final Vector2d thirdSamplePosition = new Vector2d(-23.85, 7.825);
    private final Pose2d parkPosition = new Pose2d(-13.6212, 7.5, (Math.PI / 2)+0.2);
    public void buildTrajectories() {
        preloadOuttake = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(highBucketPosition)
                .build();
        firstIntake = drive.trajectoryBuilder(preloadOuttake.end())
                .splineTo(firstSamplePosition, 0.75)
                .build();
        firstOuttake = drive.trajectoryBuilder(firstIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        secondIntake = drive.trajectoryBuilder(firstOuttake.end())
                .splineTo(secondSamplePosition, 1.2)
                .build();
        secondOuttake = drive.trajectoryBuilder(secondIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        thirdIntake = drive.trajectoryBuilder(secondOuttake.end())
                .splineTo(thirdSamplePosition, 1.45)
                .build();
        thirdOuttake = drive.trajectoryBuilder(thirdIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        park = drive.trajectoryBuilder(thirdOuttake.end())
                .lineToSplineHeading(parkPosition)
                .build();
    }
    State curState = State.idle;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        buildTrajectories();
        robot.intakePivot.flipBack();
        robot.outtakePivot.flipFront();
        robot.horizontalSlide.retractFull();
        robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.55);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        timer.reset();

        drive.followTrajectoryAsync(preloadOuttake);
        curState = State.preloadOuttake;

        while(opModeIsActive()) {
            stateMachine();
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("Current State", curState.name());
            telemetry.addData("Current Time", timer.seconds());
            telemetry.addData("Left Vertical", robot.verticalSlide.getLeftPosition());
            telemetry.addData("Right Vertical", robot.verticalSlide.getRightPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private void stateMachine() throws InterruptedException {
        switch(curState) {
            case idle:
                break;
            case preloadOuttake:

                break;
            case firstIntake:

                break;
            case firstOuttake:

                break;

            case secondIntake:

                break;
            case secondOuttake:

                break;
            case thirdIntake:

                break;
            case thirdOuttake:

                break;
            case park:

                break;

        }
    }

//    private double positionDifference(Pose2d currentPosition, Pose2d targetPosition) {
//        double distance = Math.hypot(
//                targetPosition.getX() - currentPosition.getX(),
//                targetPosition.getY() - currentPosition.getY()
//        );
//        return distance;
//    }
//    private boolean isAt(Pose2d currentPosition, Pose2d targetPosition) {
//        boolean positionDifference = isAt(currentPosition, targetPosition, tolerance);
//        double headingDifference = Math.abs(targetPosition.getHeading() - currentPosition.getHeading());
//        headingDifference = (headingDifference + Math.PI) % (2 * Math.PI) - Math.PI;
//        return positionDifference && Math.abs(headingDifference) <= headingTolerance;
//    }

}