package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class FiveSpecimenSolo extends LinearOpMode {
    Robot robot;


    // 1225
    public int verticalSlidePosition = 0;
    final double SPECIMEN_GRAB_POSITION = 0.325;
    final double SPECIMEN_SCORE_POSITION = 0.3;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 0.9;
    final int OUTTAKE_SLIDE_POSITION = 1070;

    private SampleMecanumDrive drive;
    public Trajectory presetSpecimenScore, firstSampleIntake, firstSampleOuttake, secondSampleIntake, secondSampleOuttake, thirdSampleIntake, thirdSampleIntakePush, thirdSampleOuttake, firstSpecimenIntake, firstSpecimenOuttake, firstSpecimenOuttakeClip, secondSpecimenIntake, secondSpecimenOuttake, secondSpecimenOuttakeClip, thirdSpecimenIntake, thirdSpecimenOuttake, thirdSpecimenOuttakeClip, fourthSpecimenIntake, fourthSpecimenOuttake, fourthSpecimenOuttakeClip;
    enum State {
        idle,
        presetSpecimenPositions,
        presetSpecimenScore,
        firstSampleIntake,
        firstSampleOuttake,
        secondSampleIntake,
        secondSampleOuttake,
        thirdSampleIntake,
        thirdSampleIntakePush,
        thirdSampleOuttake,
        firstSpecimenIntake,
        firstSpecimenOuttake,
        firstSpecimenOuttakeClip,
        secondSpecimenIntake,
        secondSpecimenOuttake,
        secondSpecimenOuttakeClip,
        thirdSpecimenIntake,
        thirdSpecimenOuttake,
        thirdSpecimenOuttakeClip,
        fourthSpecimenIntake,
        fourthSpecimenOuttake,
        fourthSpecimenOuttakeClip,
    }
    State curState = State.idle;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.intakePivot.flipTo(0.2);
        robot.outtakePivot.flipFront();
        robot.horizontalSlide.retractFull();
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.5);
        robot.hangPivot.setPosition(0.5);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, 0));
        presetSpecimenScore = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-30.9025, -13.7387, 0.1551))
                .build();
        firstSampleIntake = drive.trajectoryBuilder(presetSpecimenScore.end())
                .splineTo(new Vector2d(-29.9995, 17.0929), 2.2429)
                .build();
        firstSampleOuttake = drive.trajectoryBuilder(firstSampleIntake.end())
                .lineToLinearHeading(new Pose2d(-16.0643, 18.0141, 1.2072))
                .build();
        secondSampleIntake = drive.trajectoryBuilder(firstSampleOuttake.end())
                .lineToLinearHeading(new Pose2d(-27.25, 29.75, 2.4988))
                .build();
        secondSampleOuttake = drive.trajectoryBuilder(secondSampleIntake.end())
                .lineToLinearHeading(new Pose2d(-16.5414, 26.9298, 0.9341))
                .build();
        thirdSampleIntake = drive.trajectoryBuilder(secondSampleOuttake.end())
                .lineToLinearHeading(new Pose2d(-31.1305, 36.2107, 2.2585))
                .build();
        thirdSampleIntakePush = drive.trajectoryBuilder(thirdSampleIntake.end())
                .lineToLinearHeading(new Pose2d(-31.1305, 38.2107, 2.2585))
                .build();
        thirdSampleOuttake = drive.trajectoryBuilder(thirdSampleIntakePush.end())
                .lineToLinearHeading(new Pose2d(-15.861, 36.1574, 0.8096))
                .build();
        firstSpecimenIntake = drive.trajectoryBuilder(thirdSampleOuttake.end())
                .lineToLinearHeading(new Pose2d(-15.9803, 11.7503, 1.2907))
                .build();
        firstSpecimenOuttake = drive.trajectoryBuilder(firstSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-15.9118, -12.7387, 0.1551))
                .build();
        firstSpecimenOuttakeClip = drive.trajectoryBuilder(firstSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-29.9118, -12.7387, 0.1551))
                .build();
        secondSpecimenIntake = drive.trajectoryBuilder(firstSpecimenOuttakeClip.end())
                .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                .build();
        secondSpecimenOuttake = drive.trajectoryBuilder(secondSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-15.9118, -8.7387, 0.1551))
                .build();
        secondSpecimenOuttakeClip = drive.trajectoryBuilder(secondSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-29.9118, -8.7387, 0.1551))
                .build();
        thirdSpecimenIntake = drive.trajectoryBuilder(secondSpecimenOuttakeClip.end())
                .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                .build();
        thirdSpecimenOuttake = drive.trajectoryBuilder(thirdSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-15.9118, -5.7387, 0.1551))
                .build();
        thirdSpecimenOuttakeClip = drive.trajectoryBuilder(thirdSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-30.9118, -5.7387, 0.1551))
                .build();
        fourthSpecimenIntake = drive.trajectoryBuilder(secondSpecimenOuttakeClip.end())
                .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                .build();
        fourthSpecimenOuttake = drive.trajectoryBuilder(thirdSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-15.9118, -2.7387, 0.1551))
                .build();
        fourthSpecimenOuttakeClip = drive.trajectoryBuilder(thirdSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-30.9118, -2.7387, 0.1551))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        timer.reset();

        curState = State.presetSpecimenPositions;

        while(opModeIsActive()) {
            stateMachine();
            robot.verticalSlide.goTo(verticalSlidePosition, 1);
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("Vertical Slide", verticalSlidePosition);
            telemetry.addData("Current State", curState.name());
            telemetry.addData("Current Time", timer.seconds());
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
            case presetSpecimenPositions:
                if(timer.seconds()>0 && timer.seconds()<0.5){
                    verticalSlidePosition = OUTTAKE_SLIDE_POSITION;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>0.5) {
                    timer.reset();
                    drive.followTrajectoryAsync(presetSpecimenScore);
                    curState = State.presetSpecimenScore;
                }
                break;
            case presetSpecimenScore:
                if(timer.seconds()>1.4 && timer.seconds()<1.5){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>1.5) {
                    robot.outtakePivot.flipFront();
                    timer.reset();
                    firstSampleIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(-29.9995, 17.0929), 2.2429)
                            .build();
                    drive.followTrajectoryAsync(firstSampleIntake);
                    curState = State.firstSampleIntake;
                }
                break;
            case firstSampleIntake:
                if(timer.seconds() > 0 && timer.seconds() < 1) {
                    robot.intakePivot.flipTo(0.6);
                    robot.horizontalSlide.extendFull();
                    robot.spintake.spinIn(1);
                    robot.clawPivot.flipTo(0.1);
                }
                if(timer.seconds() > 1.5 && timer.seconds() < 1.7) {
                    robot.intakePivot.flipTo(0.74);
                }
                if(!drive.isBusy() && timer.seconds() > 1.7) {
                    timer.reset();
                    drive.followTrajectoryAsync(firstSampleOuttake);
                    curState = State.firstSampleOuttake;
                }
                break;
            case firstSampleOuttake:
                if(timer.seconds() > 1 && timer.seconds() < 1.2) {
                    robot.clawPivot.flipTo(0.5);
                }
                if(timer.seconds() > 1 && timer.seconds() < 1.5) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds()>1.5) {
                    robot.intakePivot.flipTo(0.6);
                    timer.reset();
                    secondSampleIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-27.25, 29.75, 2.4988))
                            .build();
                    drive.followTrajectoryAsync(secondSampleIntake);
                    curState = State.secondSampleIntake;
                }
                break;
            case secondSampleIntake:
                if(timer.seconds() > 0 && timer.seconds() < 1.5) {
                    robot.spintake.spinIn(1);
                    robot.clawPivot.flipTo(0.1);
                    robot.horizontalSlide.extendFull();
                }
                if(timer.seconds() > 1.5 && timer.seconds() < 1.7) {
                    robot.intakePivot.flipTo(0.74);
                }
                if(!drive.isBusy() && timer.seconds() > 1.7) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(secondSampleOuttake);
                    curState = State.secondSampleOuttake;
                }
                break;
            case secondSampleOuttake:
                if(timer.seconds() > 0.1 && timer.seconds() < 1) {
                    robot.horizontalSlide.retractFull();
                }
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.clawPivot.flipTo(0.5);
                }
                if(timer.seconds() > 1 && timer.seconds() < 1.5) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds()>1.5) {
                    robot.intakePivot.flipTo(0.26);
                    timer.reset();
                    thirdSampleIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-31.1305, 36.2107, 2.2585))
                            .build();
                    drive.followTrajectoryAsync(thirdSampleIntake);
                    curState = State.thirdSampleIntake;
                }
                break;
            case thirdSampleIntake:
                if(timer.seconds() > 0 && timer.seconds() < 1) {
                    robot.spintake.spinIn(1);
                    robot.clawPivot.flipTo(0.1);
                    robot.horizontalSlide.extendFull();
                }
                if(timer.seconds() > 1.1 && timer.seconds() < 1.5) {
                    robot.intakePivot.flipTo(0.74);
                }
                if(!drive.isBusy() && timer.seconds()>1.5) {
                    timer.reset();
                    drive.followTrajectoryAsync(thirdSampleIntakePush);
                    curState = State.thirdSampleIntakePush;
                }
                break;
            case thirdSampleIntakePush:
                if(timer.seconds()>0.5) {
                    timer.reset();
                    drive.followTrajectoryAsync(thirdSampleOuttake);
                    curState = State.thirdSampleOuttake;
                }
                break;
            case thirdSampleOuttake:
                if(timer.seconds() > 0 && timer.seconds() < 1.2) {
                    robot.horizontalSlide.retractFull();
                }
                if(timer.seconds() > 1 && timer.seconds() < 1.2) {
                    robot.clawPivot.flipTo(0.5);
                }
                if(timer.seconds() > 1.2 && timer.seconds() < 1.5) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds() > 1.5) {
                    robot.horizontalSlide.goTo(0.5);
                    timer.reset();
                    firstSpecimenIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.2907))
                            .build();
                    drive.followTrajectoryAsync(firstSpecimenIntake);
                    curState = State.firstSpecimenIntake;
                }
                break;
            case firstSpecimenIntake:
                if(timer.seconds() > 0.8 && timer.seconds() < 1.3) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.clawPivot.flipTo(0.5);
                    robot.spintake.spinIn(1);
                    robot.intakePivot.flipTo(0.73);
                }
                if(timer.seconds() > 1.3 && timer.seconds() < 2){
                    robot.horizontalSlide.goTo(1);
                }
                if(!drive.isBusy() && timer.seconds() > 2) {
                    timer.reset();
                    drive.followTrajectoryAsync(firstSpecimenOuttake);
                    curState = State.firstSpecimenOuttake;
                }
                break;
            case firstSpecimenOuttake:
                if(timer.seconds() > 0 && timer.seconds() < 0.6) {
                    robot.intakePivot.flipTo(0.26);
                    robot.horizontalSlide.retractFull();
                    robot.outtakePivot.flipFront();
                }
                if(timer.seconds() > 0.7 && timer.seconds() < 0.8) {
                    robot.outtakeClaw.grab();
                }
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.spintake.spinOut(1);
                    robot.intakePivot.flipTo(0.26);
                }
                if(timer.seconds()>1 && timer.seconds()<1.5){
                    verticalSlidePosition = OUTTAKE_SLIDE_POSITION;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds() > 1.5) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(firstSpecimenOuttakeClip);
                    curState = State.firstSpecimenOuttakeClip;
                }
                break;
            case firstSpecimenOuttakeClip:
                if(timer.seconds()>0.5 && timer.seconds()<0.6){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>0.6) {
                    timer.reset();
                    robot.outtakePivot.flipFront();
                    secondSpecimenIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                            .build();
                    drive.followTrajectoryAsync(secondSpecimenIntake);
                    curState = State.secondSpecimenIntake;
                }
                break;
            case secondSpecimenIntake:
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.clawPivot.flipTo(0.5);
                    robot.spintake.spinIn(1);
                    robot.intakePivot.flipTo(0.73);
                }
                if(timer.seconds() > 1.5 && timer.seconds() < 2.2){
                    robot.horizontalSlide.goTo(1);
                }
                if(!drive.isBusy() && timer.seconds() > 2.2) {
                    timer.reset();
                    drive.followTrajectoryAsync(secondSpecimenOuttake);
                    curState = State.secondSpecimenOuttake;
                }
                break;
            case secondSpecimenOuttake:
                if(timer.seconds() > 0 && timer.seconds() < 0.6) {
                    robot.intakePivot.flipTo(0.26);
                    robot.horizontalSlide.retractFull();
                    robot.outtakePivot.flipFront();
                }
                if(timer.seconds() > 0.7 && timer.seconds() < 0.8) {
                    robot.outtakeClaw.grab();
                }
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.spintake.spinOut(1);
                    robot.intakePivot.flipTo(0.26);
                }
                if(timer.seconds()>1 && timer.seconds() < 1.5){
                    verticalSlidePosition = OUTTAKE_SLIDE_POSITION;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds() > 1.5) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(secondSpecimenOuttakeClip);
                    curState = State.secondSpecimenOuttakeClip;
                }
                break;
            case secondSpecimenOuttakeClip:
                if(timer.seconds()>0.5 && timer.seconds()<0.6){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>0.6) {
                    timer.reset();
                    robot.outtakePivot.flipFront();
                    thirdSpecimenIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                            .build();
                    drive.followTrajectoryAsync(thirdSpecimenIntake);
                    curState = State.thirdSpecimenIntake;
                }
                break;
            case thirdSpecimenIntake:
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.clawPivot.flipTo(0.5);
                    robot.spintake.spinIn(1);
                    robot.intakePivot.flipTo(0.73);
                }
                if(timer.seconds() > 1.5 && timer.seconds() < 2.2){
                    robot.horizontalSlide.goTo(1);
                }
                if(!drive.isBusy() && timer.seconds() > 2.2) {
                    timer.reset();
                    drive.followTrajectoryAsync(thirdSpecimenOuttake);
                    curState = State.thirdSpecimenOuttake;
                }
                break;
            case thirdSpecimenOuttake:
                if(timer.seconds() > 0 && timer.seconds() < 0.6) {
                    robot.intakePivot.flipTo(0.26);
                    robot.horizontalSlide.retractFull();
                    robot.outtakePivot.flipFront();
                }
                if(timer.seconds() > 0.7 && timer.seconds() < 0.8) {
                    robot.outtakeClaw.grab();
                }
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.spintake.spinOut(1);
                    robot.intakePivot.flipTo(0.26);
                }
                if(timer.seconds()>1 && timer.seconds() < 1.5){
                    verticalSlidePosition = OUTTAKE_SLIDE_POSITION;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds() > 1.5) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(thirdSpecimenOuttakeClip);
                    curState = State.thirdSpecimenOuttakeClip;
                }
                break;
            case thirdSpecimenOuttakeClip:
                if(timer.seconds()>0.6 && timer.seconds()<0.7){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>0.7) {
                    timer.reset();
                    robot.outtakePivot.flipFront();
                    fourthSpecimenIntake = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-15.0803, 11.0503, 1.3007))
                            .build();
                    drive.followTrajectoryAsync(fourthSpecimenIntake);
                    curState = State.fourthSpecimenIntake;
                }
                break;
            case fourthSpecimenIntake:
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.clawPivot.flipTo(0.5);
                    robot.spintake.spinIn(1);
                    robot.intakePivot.flipTo(0.73);
                }
                if(timer.seconds() > 1.5 && timer.seconds() < 2.2) {
                    robot.horizontalSlide.goTo(1);
                }
                if(!drive.isBusy() && timer.seconds() > 2.2) {
                    timer.reset();
                    drive.followTrajectoryAsync(fourthSpecimenOuttake);
                    curState = State.fourthSpecimenOuttake;
                }
                break;
            case fourthSpecimenOuttake:
                if(timer.seconds() > 0 && timer.seconds() < 0.6) {
                    robot.intakePivot.flipTo(0.26);
                    robot.horizontalSlide.retractFull();
                    robot.outtakePivot.flipFront();
                }
                if(timer.seconds() > 0.7 && timer.seconds() < 0.8) {
                    robot.outtakeClaw.grab();
                }
                if(timer.seconds() > 0.8 && timer.seconds() < 1) {
                    robot.horizontalSlide.goTo(0.5);
                    robot.spintake.spinOut(1);
                    robot.intakePivot.flipTo(0.26);
                }
                if(timer.seconds()>1 && timer.seconds() < 1.5){
                    verticalSlidePosition = OUTTAKE_SLIDE_POSITION;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds() > 1.5) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(fourthSpecimenOuttakeClip);
                    curState = State.fourthSpecimenOuttakeClip;
                }
                break;
            case fourthSpecimenOuttakeClip:
                if(timer.seconds()>0.6 && timer.seconds()<0.7){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>0.7) {
                    timer.reset();
                    robot.outtakePivot.flipFront();
                    curState = State.idle;
                }
                break;
        }
    }

}