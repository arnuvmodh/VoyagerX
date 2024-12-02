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

import java.util.Arrays;

@Autonomous()
public class Specimen extends LinearOpMode {
    private Robot robot;
    int verticalSlidePosition = 0;
    final double SPECIMEN_GRAB_POSITION = 0.2;
    final double SPECIMEN_SCORE_POSITION = 0.65;
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 1;
    final double INTAKE_CLAW_CLOSE_POSITION = 0.55;
    private SampleMecanumDrive drive;
    public Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9;
    enum State {
        idle,
        traj1,
        traj2,
        traj3,
        traj4,
        traj5,
        traj6,
        traj7,
        traj8,
        traj9
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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, 0));
        traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-30, -2, 6.2221))
                .build();
        traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-17.5, 39.6287), 3.1287)
                .build();
        traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-18.25, 49.6687))
                .build();
        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-5.5, 24.9))
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-26, -4, 0))
                .build();
        traj6 = drive.trajectoryBuilder(traj5.end())
                .splineToSplineHeading(new Pose2d(-10, 24.9, 3.1167), Math.PI)
                .splineToSplineHeading(new Pose2d(-5.5, 24.9, 3.1167), Math.PI)
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-26.25, -7, 0))
                .build();
        traj8 = drive.trajectoryBuilder(traj5.end())
                .splineToSplineHeading(new Pose2d(-10, 24.9, 3.1167), Math.PI)
                .splineToSplineHeading(new Pose2d(-5.5, 24.9, 3.1167), Math.PI)
                .build();
        traj9 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-27.25, 3, 0))
                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        timer.reset();

        drive.followTrajectoryAsync(traj1);
        curState = State.traj1;

        while(opModeIsActive()) {
            stateMachine();
            robot.verticalSlide.goTo(verticalSlidePosition, 1);
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("Current State", curState.name());
            telemetry.addData("Current Time", timer.seconds());
            telemetry.addData("Left Vertical", robot.verticalSlide.getLeftPosition());
            telemetry.addData("Right Vertical", robot.verticalSlide.getRightPosition());
            telemetry.addData("Vertical Position", verticalSlidePosition);
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
            case traj1:  //scores first specimen
                if(timer.seconds()>0 && timer.seconds()<1.8){
                    verticalSlidePosition = 800;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>1.8 && timer.seconds()<2.65){
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>2.65) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                if(!drive.isBusy() && timer.seconds()>2.7) {
                    robot.outtakePivot.flipFront();
                    timer.reset();
                    drive.followTrajectoryAsync(traj2);
                    curState = State.traj2;
                }
                break;
            case traj2:  //grabs first sample
                if(timer.seconds()>0&&timer.seconds()<3.7) {
                    robot.horizontalSlide.retractFull();
                }
                if(timer.seconds() > 1 && timer.seconds()<2.3){
                    robot.clawPivot.flipTo(0.55);
                    robot.outtakeClaw.open();
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION+0.2);
                    robot.intakePivot.flipFront();
                }
                if(timer.seconds()>2.4 && timer.seconds()<2.8){
                    robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds()>2.8&&timer.seconds()<3.4){
                    robot.intakePivot.flipBack();
                }
                if(!drive.isBusy() && timer.seconds()>3.7) {
                    timer.reset();
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
                    drive.followTrajectory(traj3);
                    curState = State.traj3;
                }
                break;
            case traj3: // outtakes first sample and intakes+outtakes 2nd sample
                if(timer.seconds()>0&&timer.seconds()<5.4) {
                    robot.horizontalSlide.retractFull();
                }
                if(timer.seconds() > 0.2 && timer.seconds() < 1.5){
                    robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION-0.2);
                }
                if(timer.seconds() > 1.4 && timer.seconds() < 1.7){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                }
                if(timer.seconds() > 1.7 && timer.seconds() < 2.4){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                }
                if(timer.seconds()>1.7&&timer.seconds()<2.4) {
                    robot.clawPivot.flipTo(0.55);
                    robot.outtakePivot.flipFront();
                    robot.intakePivot.flipFront();
                }
                if(timer.seconds() > 2.3 && timer.seconds() < 2.6){
                    robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds() > 2.6 && timer.seconds() < 3.2){
                    robot.intakePivot.flipBack();
                }
                if(timer.seconds() > 3.5 && timer.seconds() < 3.7){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds() > 3.7 && timer.seconds() < 3.9){
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
                }
                if(timer.seconds() > 3.9 && timer.seconds() < 4.3){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                }
                if(timer.seconds() > 4.3 && timer.seconds() < 4.4){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                }
                if(!drive.isBusy() && timer.seconds() > 4.4 && timer.seconds() < 5.4){
                    timer.reset();
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION + 0.1);
                    drive.followTrajectory(traj4);
                    curState = State.traj4;
                }
                break;

            case traj4:  //grabs second specimen off of wall
                if(timer.seconds()>0&&timer.seconds()<1.8) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION+0.1);
                }
                if(timer.seconds()>1.8&&timer.seconds()<2) {
                    verticalSlidePosition = 1400;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION-0.2);
                }
                if(!drive.isBusy()&&timer.seconds()>2.1&&timer.seconds()<3.7) {
                    timer.reset();
                    drive.followTrajectory(traj5);
                    curState = State.traj5;
                }
                break;
            case traj5: //hangs second specimen
                if(timer.seconds()>0 && timer.seconds()<2.5){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                }
                if(timer.seconds()>2.6&&timer.seconds()<3){
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>3.8&&timer.seconds()<3.9) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                if(!drive.isBusy()&&timer.seconds()>3.9) {
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                    timer.reset();
                    drive.followTrajectory(traj6);
                    curState = State.traj6;
                }
                break;
            case traj6: //grabs 3rd specimen off of wall
                if(!robot.outtakeClaw.isAt(OUTTAKE_CLAW_CLOSE_POSITION+0.1)){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION+0.2);
                }
                if(timer.seconds()>2.3&&timer.seconds()<2.7) {
                    verticalSlidePosition = 1400;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION-0.2);
                }
                if(!drive.isBusy()&&timer.seconds()>2.8&&timer.seconds()<4) {
                    timer.reset();
                    drive.followTrajectory(traj7);
                    curState = State.traj7;
                }
                break;
            case traj7: //hangs 3rd specimen
                if(timer.seconds()>0 && timer.seconds()<2.5){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                }
                if(timer.seconds()>2.6&&timer.seconds()<3){
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>3.8&&timer.seconds()<3.9) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                if(!drive.isBusy()&&timer.seconds()>3.9) {
                    timer.reset();
                    drive.followTrajectory(traj8);
                    curState = State.traj8;
                }
                break;
            case traj8:
                if(!robot.outtakeClaw.isAt(OUTTAKE_CLAW_CLOSE_POSITION+0.1)){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION+0.1);
                }
                if(timer.seconds()>2.1&&timer.seconds()<2.8) {
                    verticalSlidePosition = 1500;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(!drive.isBusy()&&timer.seconds()>2.8&&timer.seconds()<4) {
                    timer.reset();
                    drive.followTrajectory(traj9);
                    curState = State.traj9;
                }
                break;
            case traj9:
                if(timer.seconds()>0 && timer.seconds()<2.4){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                }
                if(timer.seconds()>2.4&&timer.seconds()<3){
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>3.8&&timer.seconds()<3.9) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                break;

        }
    }



    public void transfer() {
        robot.outtakeClaw.close();
        if (robot.outtakeClaw.isClosed()) {
            robot.intakeClaw.open();
        }
    }

    public void grabSample() {
        robot.outtakeClaw.open();
        robot.intakeClaw.open();
        robot.intakePivot.flipBack();
        // robot.horizontalSlide.extend(0.1);
        if (robot.intakePivot.isOut()) {
            robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION);
            if (robot.intakeClaw.isClosed()) {
                robot.intakePivot.flipFront();
            }
        }
    }

    public void grabSpecimen() {
        robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
        if (!robot.outtakePivot.isAt(SPECIMEN_GRAB_POSITION)) {
            return;
        }
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
    }

    public void scoreSpecimen() {
        robot.verticalSlide.extendFull();
        robot.verticalSlide.goTo(1600, 1);
        if(!robot.verticalSlide.isAt(1600)) return;
        robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
        if(!robot.outtakePivot.isAt(SPECIMEN_SCORE_POSITION)) return;
        robot.verticalSlide.goTo(800,1);
        if(!robot.verticalSlide.isAt(800)) return;
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
        if(!robot.outtakeClaw.isAt(OUTTAKE_CLAW_OPEN_POSITION)) return;
        robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
        if(!robot.outtakePivot.isAt(SPECIMEN_GRAB_POSITION)) return;
        robot.verticalSlide.retractFull();
    }

}