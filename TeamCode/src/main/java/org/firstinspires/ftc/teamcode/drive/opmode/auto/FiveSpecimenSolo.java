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
    final double SPECIMEN_SCORE_POSITION = 0.25;;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 0.9;
    final int OUTTAKE_SLIDE_POSITION = 1225;

    private SampleMecanumDrive drive;
    public Trajectory presetSpecimenScore, firstSpecimenIntake, firstSpecimenOuttake, secondSpecimenIntake, secondSpecimenOuttake, thirdSpecimenIntake, thirdSpecimenOuttake;
    enum State {
        idle,
        presetSpecimenPositions,
        presetSpecimenScore,
        firstSpecimenIntake,
        firstSpecimenOuttake,
        secondSpecimenIntake,
        secondSpecimenOuttake,
        thirdSpecimenIntake,
        thirdSpecimenOuttake
    }
    State curState = State.idle;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.intakePivot.flipBack();
        robot.outtakePivot.flipFront();
        robot.horizontalSlide.retractFull();
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.5);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, 0));
        presetSpecimenScore = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-29.9025, -3.7387, 0.0261))
                .build();
        firstSpecimenIntake = drive.trajectoryBuilder(presetSpecimenScore.end())
                .splineTo(new Vector2d(-19, 22.1931), 2.3327)
                .build();
        firstSpecimenOuttake = drive.trajectoryBuilder(firstSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-11.3872, 23.3132, 1.1445))
                .build();
        secondSpecimenIntake = drive.trajectoryBuilder(firstSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-23.7563, 31.0475, 2.0586))
                .build();
        secondSpecimenOuttake = drive.trajectoryBuilder(secondSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-13.3346, 35.4988, 0.9066))
                .build();
        thirdSpecimenIntake = drive.trajectoryBuilder(secondSpecimenOuttake.end())
                .lineToLinearHeading(new Pose2d(-24.2928, 36.8494, 1.9469))
                .build();
        thirdSpecimenOuttake = drive.trajectoryBuilder(thirdSpecimenIntake.end())
                .lineToLinearHeading(new Pose2d(-12.9646, 39.2893, 0.8409))
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
            robot.horizontalSlide.retractFull();
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
                if(!drive.isBusy() && timer.seconds()>0.5) {
                    timer.reset();
                    drive.followTrajectoryAsync(presetSpecimenScore);
                    curState = State.presetSpecimenScore;
                }
                break;
            case presetSpecimenScore:
                if(timer.seconds()>1.8 && timer.seconds()<2){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    verticalSlidePosition = 0;
                }
                if(!drive.isBusy() && timer.seconds()>2) {
                    timer.reset();
                    drive.followTrajectoryAsync(firstSpecimenIntake);
                    curState = State.firstSpecimenIntake;
                }
                break;
            case firstSpecimenIntake:
                if(timer.seconds() > 1) {
                    robot.intakePivot.flipTo(0.73);
                }
                if(timer.seconds() > 2) {
                    robot.horizontalSlide.extendFull();
                    robot.spintake.spinIn(1);
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    robot.horizontalSlide.retractFull();
                    drive.followTrajectoryAsync(firstSpecimenOuttake);
                    curState = State.firstSpecimenOuttake;
                }
                break;
            case firstSpecimenOuttake:
                if(timer.seconds() > 2) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    drive.followTrajectoryAsync(secondSpecimenIntake);
                    curState = State.secondSpecimenIntake;
                }
                break;
            case secondSpecimenIntake:
                if(timer.seconds() > 2) {
                    robot.horizontalSlide.extendFull();
                    robot.spintake.spinIn(1);
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    drive.followTrajectoryAsync(secondSpecimenOuttake);
                    curState = State.secondSpecimenOuttake;
                }
                break;
            case secondSpecimenOuttake:
                if(timer.seconds() > 2) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    drive.followTrajectoryAsync(thirdSpecimenIntake);
                    curState = State.thirdSpecimenIntake;
                }
                break;
            case thirdSpecimenIntake:
                if(timer.seconds() > 2) {
                    robot.horizontalSlide.extendFull();
                    robot.spintake.spinIn(1);
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    drive.followTrajectoryAsync(thirdSpecimenOuttake);
                    curState = State.thirdSpecimenOuttake;
                }
                break;
            case thirdSpecimenOuttake:
                if(timer.seconds() > 2) {
                    robot.spintake.spinOut(1);
                }
                if(!drive.isBusy() && timer.seconds()>2) {
                    timer.reset();
                    curState = State.idle;
                }
                break;
        }
    }

}