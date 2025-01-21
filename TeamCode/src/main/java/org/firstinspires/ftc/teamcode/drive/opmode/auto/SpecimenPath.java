package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class SpecimenPath extends LinearOpMode {
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
                .splineToLinearHeading(new Pose2d(-6, 28.5, 3.1626), 0)
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .splineToLinearHeading(new Pose2d(-23, -5, 6.2576), Math.PI)
                .build();
        traj8 = drive.trajectoryBuilder(traj7.end())
                .splineToLinearHeading(new Pose2d(-6, 28.5, 3.1626), 0)
                .build();
        traj9 = drive.trajectoryBuilder(traj8.end())
                .splineToLinearHeading(new Pose2d(-23, -7, 6.2576), Math.PI)
                .build();
        traj10 = drive.trajectoryBuilder(traj9.end())
                .splineToLinearHeading(new Pose2d(-6, 28.5, 3.1626), 0)
                .build();
        traj11 = drive.trajectoryBuilder(traj10.end())
                .splineToLinearHeading(new Pose2d(-23, -9, 6.2576), Math.PI)
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
                if(!drive.isBusy() && timer.seconds()>2) {
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
                    drive.followTrajectory(traj6);
                    curState = State.traj6;
                    timer.reset();
                }
                break;
            case traj6:
                if (!drive.isBusy() && timer.seconds() > 2) {
                    timer.reset();
                    drive.followTrajectory(traj7);
                    curState = State.traj7;
                }
                break;
            case traj7:
                if(!drive.isBusy()&&timer.seconds()>2) {
                    drive.followTrajectory(traj8);
                    curState = State.traj8;
                    timer.reset();
                }
                break;
            case traj8:
                if (!drive.isBusy() && timer.seconds() > 2) {
                    timer.reset();
                    drive.followTrajectory(traj9);
                    curState = State.traj9;
                }
                break;
            case traj9:
                if (!drive.isBusy() && timer.seconds() > 2) {
                    timer.reset();
                    drive.followTrajectory(traj10);
                    curState = State.traj10;
                }
                break;
            case traj10:
                if (!drive.isBusy() && timer.seconds() > 2) {
                    timer.reset();
                    drive.followTrajectory(traj11);
                    curState = State.traj11;
                }
                break;
            case traj11:
                break;
        }
    }

}