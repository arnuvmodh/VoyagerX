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
public class Specimen extends LinearOpMode {
    private Robot robot;
    final double SPECIMEN_GRAB_POSITION = 0.2;
    final double SPECIMEN_SCORE_POSITION = 0.7;
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 0.9;
    final double INTAKE_CLAW_CLOSE_POSITION = 0.55;
    private SampleMecanumDrive drive;
    public Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj7;
    enum State {
        idle,
        traj1,
        traj2,
        traj3,
        traj4,
        traj5,
        traj6,
        traj7
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
                .lineToSplineHeading(new Pose2d(-30.5, -1.25, 6.2221))
                .build();
        traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-18.9, 40), 3.1167)
                .build();
        traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-19.5, 50))
                .build();
        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-5.5, 24.9))
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-24.5, -7.25, 0.0169))
                .build();
        traj6 = drive.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(-5.5, 24.9), 3.1167)
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-24.5, -8.25, 0.0169))
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
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("Current State", curState.name());
            telemetry.addData("Current Time", timer.seconds());
            telemetry.update();
        }
    }

    private void stateMachine() throws InterruptedException {

        switch(curState) {
            case idle:
                break;
            case traj1:
                if(timer.seconds()>0 && timer.seconds()<2){
                    robot.verticalSlide.goTo(500, 1);
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(timer.seconds()>2 && timer.seconds()<2.8){
                    robot.verticalSlide.goTo(0, 1);
                }
                if(timer.seconds()>2.8) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                if(!drive.isBusy() && timer.seconds()>3) {
                    timer.reset();
                    drive.followTrajectoryAsync(traj2);
                    curState = State.traj2;
                }
                break;
            case traj2:
                if(timer.seconds() > 1 && timer.seconds()<2.7){
                    robot.outtakeClaw.open();
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipFront();
                    robot.intakePivot.flipFront();
                }
                if(timer.seconds()>2.7 && timer.seconds()<3.2){
                    robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds()>3.2&&timer.seconds()<4.2){
                    robot.horizontalSlide.retractFull();
                    robot.intakePivot.flipBack();
                }
                if(!drive.isBusy() && timer.seconds()>4.2) {
                    timer.reset();
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
                    drive.followTrajectory(traj3);
                    curState = State.traj3;
                }
                break;
            case traj3:
                if(timer.seconds()>1&&timer.seconds()<1.2){
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
                }
                if(timer.seconds()>1.2&&timer.seconds()<1.4){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                }

                if(timer.seconds() > 1.5 && timer.seconds()<2.2){
                    robot.outtakeClaw.open();
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipFront();
                    robot.intakePivot.flipFront();
                }
                if(timer.seconds()>2.2 && timer.seconds()<2.7){
                    robot.intakeClaw.openTo(INTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds()>2.7&&timer.seconds()<3.9){
                    robot.horizontalSlide.retractFull();
                    robot.intakePivot.flipBack();
                }
                if(timer.seconds()>3.9&&timer.seconds()<4.2){
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
                }
                if(timer.seconds()>4.2&&timer.seconds()<4.4){
                    robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
                }
                if(timer.seconds()>4.4&&timer.seconds()<4.6){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION);
                }
                if(timer.seconds()>4.8) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                }
                if(!drive.isBusy() && timer.seconds()>4.8) {
                    timer.reset();
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                    drive.followTrajectory(traj4);
                    curState = State.traj4;
                }
                break;

            case traj4:
                if(timer.seconds()>0&&timer.seconds()<1.9) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION+0.1);
                }
                if(timer.seconds()>1.9&&timer.seconds()<2.2) {
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(!drive.isBusy()&&timer.seconds()>2.2) {
                    timer.reset();
                    robot.verticalSlide.goTo(1500, 1);
                    drive.followTrajectory(traj5);
                    curState = State.traj5;
                }
                break;
            case traj5:
                if(timer.seconds()>0 && timer.seconds()<2.5){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                    robot.verticalSlide.goTo(1500, 1);
                }
                if(timer.seconds()>2.5&&timer.seconds()<3){
                    robot.verticalSlide.goTo(0, 1);
                }
                if(timer.seconds()>3.8) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
                if(!drive.isBusy()&&timer.seconds()>3.8) {
                    timer.reset();
                    drive.followTrajectory(traj6);
                    curState = State.traj6;
                }
                break;
            case traj6:
                if(timer.seconds()>0&&timer.seconds()<2.5) {
                    robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION+0.1);
                }
                if(timer.seconds()>2.5&&timer.seconds()<2.8) {
                    robot.outtakePivot.flipTo(SPECIMEN_SCORE_POSITION);
                }
                if(!drive.isBusy()&&timer.seconds()>2.8) {
                    timer.reset();
                    robot.verticalSlide.goTo(1500, 1);
                    drive.followTrajectory(traj7);
                    curState = State.traj7;
                }
                break;
            case traj7:
                if(timer.seconds()>0 && timer.seconds()<2.5){
                    robot.outtakePivot.flipTo(SPECIMEN_GRAB_POSITION+0.1);
                    robot.verticalSlide.goTo(1500, 1);
                }
                if(timer.seconds()>2.5&&timer.seconds()<3){
                    robot.verticalSlide.goTo(0, 1);
                }
                if(timer.seconds()>3.8) robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
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