package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class Sample extends LinearOpMode {
    private Robot robot;
    private Servo servoHang;
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 1;
    private SampleMecanumDrive drive;
    public Trajectory preloadOuttake, firstIntake, firstOuttake, secondIntake, secondOuttake, thirdIntake, thirdOuttake, park;
    public enum State {
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
    public enum OuttakeState {
        idle,
        extending,
        flippingOut,
        opening,
        flippingIn,
        closing,
        retracting
    }
    OuttakeState outtakeState = OuttakeState.idle;
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
        robot.intakePivot.flipBack();
        robot.outtakePivot.flipFront();
        robot.horizontalSlide.retractFull();
        robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.55);
        servoHang = hardwareMap.get(Servo.class, "servoHang");
        servoHang.setDirection(Servo.Direction.REVERSE);
        servoHang.setPosition(0);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, 0));
        buildTrajectories();

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
            telemetry.addData("At Target", robot.isAt(highBucketPosition, 1.5));
            telemetry.addData("Distance to Target", Math.hypot(
                    firstSamplePosition.getX() - poseEstimate.getX(),
                    firstSamplePosition.getY() - poseEstimate.getY()
            ));
            telemetry.addData("Current Pose X", poseEstimate.getX());
            telemetry.addData("Current Pose Y", poseEstimate.getY());
            telemetry.addData("Target Pose X", highBucketPosition.getX());
            telemetry.addData("Target Pose Y", highBucketPosition.getY());
            telemetry.addData("Current Pose Heading Deg", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Current Pose Heading Rad", poseEstimate.getHeading());
            telemetry.addData("Target Pose Heading Deg", Math.toDegrees(highBucketPosition.getHeading()));
            telemetry.addData("Target Pose Heading Rad", highBucketPosition.getHeading());
            telemetry.update();
        }
    }

    private void stateMachine() throws InterruptedException {
        switch(curState) {
            case idle:
                break;
            case preloadOuttake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(firstIntake);
                    curState = State.firstIntake;
                }
                break;
            case firstIntake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(firstOuttake);
                    curState = State.firstOuttake;
                }
                break;
            case firstOuttake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(secondIntake);
                    curState = State.secondIntake;
                }
                break;
            case secondIntake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(secondOuttake);
                    curState = State.secondOuttake;
                }
                break;
            case secondOuttake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(thirdIntake);
                    curState = State.thirdIntake;
                }
                break;
            case thirdIntake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(thirdOuttake);
                    curState = State.thirdOuttake;
                }
                break;
            case thirdOuttake:
                if(!drive.isBusy()) {
                    drive.followTrajectoryAsync(park);
                    curState = State.park;
                }
                break;
            case park:
                break;

        }
    }
    private void outtake() {
        switch(outtakeState) {
            case idle:
                break;
            case extending:
                if(robot.verticalSlide.isFullyExtended()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.verticalSlide.extendFull();
                }
                break;
            case flippingOut:
                if(robot.outtakePivot.isOut()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.outtakePivot.flipBack();
                }
                break;
            case opening:
                if(robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                }
                break;
            case flippingIn:
                if(robot.intakePivot.isIn()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.intakePivot.flipFront();
                }
                break;
            case closing:
                if(robot.outtakeClaw.isAt(OUTTAKE_CLAW_CLOSE_POSITION)) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
                }
                break;
            case retracting:
                if(robot.verticalSlide.isFullyRetracted()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.verticalSlide.extendFull();
                }
                break;
        }
    }

    private void scoreBasket() {
        switch(outtakeState) {
            case idle:
                break;
            case extending:
                if(robot.verticalSlide.isFullyExtended()) {
                    outtakeState = OuttakeState.flippingOut;
                }
                else {
                    robot.verticalSlide.extendFull();
                }
                break;
            case flippingOut:
                if(robot.outtakePivot.isOut()) {
                    outtakeState = OuttakeState.opening;
                }
                else {
                    robot.outtakePivot.flipBack();
                }
                break;
            case opening:
                if(robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) {
                    outtakeState = OuttakeState.flippingIn;
                }
                else {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                }
                break;
            case flippingIn:
                if(robot.intakePivot.isIn()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.intakePivot.flipFront();
                }
                break;
            case retracting:
                if(robot.verticalSlide.isFullyRetracted()) {
                    outtakeState = OuttakeState.idle;
                }
                else {
                    robot.verticalSlide.extendFull();
                }
                break;
        }
    }

}