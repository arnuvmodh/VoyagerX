package org.firstinspires.ftc.teamcode.drive.opmode.auto.time;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.Specimen;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class FourSpecimen extends LinearOpMode {
    Robot robot;

    public int verticalSlidePosition = 0;
    final double SPECIMEN_GRAB_POSITION = 0.325;
    final double SPECIMEN_SCORE_POSITION = 0.65;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 0.9;

    private SampleMecanumDrive drive;
    public Trajectory traj1, traj2, traj2cont, traj3, traj3cont, traj4, traj5, traj5cont, traj6, traj7, traj8, traj9, traj10, traj11;
    enum State {
        idle,
        traj1,
        traj2,
        traj2cont,
        traj3cont,
        traj3,
        traj4,
        traj5,
        traj5cont,
        traj6,
        traj7,
        traj8,
        traj9,
        traj10,
        traj11
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
        traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-30, -2, 6.2221))
                .build();
        //first push
        traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-20, 26), Math.PI)
                .build();
        traj2cont = drive.trajectoryBuilder((traj2.end()))
                .strafeTo(new Vector2d(-55, 26))
                .build();
        traj3 = drive.trajectoryBuilder(traj2cont.end())
                .strafeTo(new Vector2d(-55, 35))
                .build();
        traj3cont = drive.trajectoryBuilder(traj3.end())
                .strafeTo(new Vector2d(-15, 35))
                .build();
        //second push
        traj4 = drive.trajectoryBuilder(traj3cont.end())
                .splineTo(new Vector2d(-55, 35), Math.PI)
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeTo(new Vector2d(-55, 45))
                .build();
        traj5cont = drive.trajectoryBuilder(traj5.end())
                .strafeTo(new Vector2d(-15, 45))
                .build();

        traj6 = drive.trajectoryBuilder(traj5cont.end())
                .splineToLinearHeading(new Pose2d(-7.5, 28.5, Math.PI), 0)
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .splineToLinearHeading(new Pose2d(-28.25, -5, (2*Math.PI)-0.1), Math.PI-0.1)
                .build();
        traj8 = drive.trajectoryBuilder(traj7.end())
                .splineToLinearHeading(new Pose2d(-10, 28.5, Math.PI), 0)
                .build();
        traj9 = drive.trajectoryBuilder(traj8.end())
                .splineToLinearHeading(new Pose2d(-28, -9, (2*Math.PI)-0.1), Math.PI-0.1)
                .build();
        traj10 = drive.trajectoryBuilder(traj9.end())
                .splineToLinearHeading(new Pose2d(-9, 28.5, Math.PI), 0)
                .build();
        traj11 = drive.trajectoryBuilder(traj10.end())
                .splineToLinearHeading(new Pose2d(-28, -13, (2*Math.PI)-0.1), Math.PI-0.1)
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
            case traj2:
                if(!drive.isBusy()) {
                    timer.reset();
                    drive.followTrajectory(traj2cont);
                    curState = State.traj2cont;
                }
                break;
            case traj2cont:
                if(!drive.isBusy()) {
                    timer.reset();
                    drive.followTrajectory(traj3);
                    curState = State.traj3;
                }
            case traj3:
                if(!drive.isBusy()){
                    drive.followTrajectory(traj3cont);
                    curState = State.traj3cont;
                    timer.reset();
                }
                break;
            case traj3cont:
                if(!drive.isBusy()){
                    drive.followTrajectory(traj4);
                    curState = State.traj4;
                    timer.reset();
                }
                break;
            case traj4:
                if (!drive.isBusy()) {
                    timer.reset();
                    drive.followTrajectory(traj5);
                    curState = State.traj5;
                }
                break;
            case traj5:
                if(!drive.isBusy()) {
                    drive.followTrajectory(traj5cont);
                    curState = State.traj5cont;
                    timer.reset();
                }
                break;
            case traj5cont:
                if(!drive.isBusy()) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                    drive.followTrajectory(traj6);
                    curState = State.traj6;
                    timer.reset();
                }
                break;
            case traj6:
                if (timer.seconds() > 0 && timer.seconds() < 0.3) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION + 0.1);
                }
                if(timer.seconds()>0.25&&timer.seconds()<0.3) {
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (timer.seconds() > 0.3 && timer.seconds() < 0.4) {
                    verticalSlidePosition = 800;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (!drive.isBusy() && timer.seconds() > 0.5) {
                    timer.reset();
                    drive.followTrajectory(traj7);
                    curState = State.traj7;
                }
                break;
            case traj7:
                if(timer.seconds()>0 && timer.seconds()<1.8){
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>1.8 && timer.seconds()<2.65){
                    verticalSlidePosition = 0;
                }
                if(!drive.isBusy() && timer.seconds()>2.8) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                    timer.reset();
                    drive.followTrajectoryAsync(traj8);
                    curState = State.traj8;
                }
                break;
            case traj8:
                if (timer.seconds() > 2 && timer.seconds() < 2.3) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION + 0.1);
                }
                if(timer.seconds()>2.25&&timer.seconds()<2.3) {
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (timer.seconds() > 2.3 && timer.seconds() < 2.5) {
                    verticalSlidePosition = 800;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (!drive.isBusy() && timer.seconds() > 2.5) {
                    timer.reset();
                    drive.followTrajectory(traj9);
                    curState = State.traj9;
                }
                break;
            case traj9:
                if(timer.seconds()>0 && timer.seconds()<1.8){
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>1.8 && timer.seconds()<2.65){
                    verticalSlidePosition = 0;
                }
                if(!drive.isBusy() && timer.seconds()>2.8) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                    timer.reset();
                    curState = State.idle;
                    drive.followTrajectoryAsync(traj10);
                    curState = State.traj10;
                }
                break;
            case traj10:
                if (timer.seconds() > 2 && timer.seconds() < 2.3) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION + 0.1);
                }
                if(timer.seconds()>2.25&&timer.seconds()<2.3) {
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (timer.seconds() > 2.3 && timer.seconds() < 2.5) {
                    verticalSlidePosition = 800;
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if (!drive.isBusy() && timer.seconds() > 2.5) {
                    timer.reset();
                    drive.followTrajectory(traj11);
                    curState = State.traj11;
                }
                break;
            case traj11:
                if(timer.seconds()>0 && timer.seconds()<1.8){
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>1.75 && timer.seconds()<2.6){
                    verticalSlidePosition = 0;
                }
                if(timer.seconds()>2 && timer.seconds()<3) {
                    robot.outtakePivot.flipBack();
                }
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    curState = State.idle;
                }
                break;
        }
    }

}