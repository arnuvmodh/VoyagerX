package org.firstinspires.ftc.teamcode.drive.opmode.auto.time.five;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.Submersible;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

@Autonomous
public class FiveSampleSolo extends LinearOpMode {

    double xPosition = 0;
    double intakeSlidePosition = 0.6;
    double intakeClawPivotPosition = 0.5;

    Pose2d getSampleFiveTrajectory() {
        return new Pose2d(17.5, 60+xPosition, 6.2498);
    }

    double getIntakeSlidePosition() {
        return intakeSlidePosition;
    }

    double getIntakeClawPivotPosition() {
        return intakeClawPivotPosition;
    }

    final double INTAKE_CLAW_OPEN_POSITION = 0.2;
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 1;
    enum State {
        traj1,
        traj2,
        traj3,
        traj4,
        traj5,
        traj6,
        traj7,
        traj8,
        traj9,
        traj10,
        traj11,
        traj12,
        idle
    }

    Robot robot;
    private SampleMecanumDrive drive;


    double increment = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean exit = false;
        boolean upDown = false, downDown = false, leftDown = false, rightDown = false;
        boolean aDown = false, bDown = false, xDown = false, yDown = false;
        while(!exit && !isStopRequested()) {
            if(gamepad1.dpad_up && !upDown) intakeSlidePosition+=(increment/10);
            upDown = gamepad1.dpad_up;
            if(gamepad1.dpad_down && !downDown) intakeSlidePosition-=(increment/10);
            downDown = gamepad1.dpad_down;
            if(gamepad1.dpad_left && !leftDown) xPosition-=increment;
            leftDown = gamepad1.dpad_left;
            if(gamepad1.dpad_right && !rightDown) xPosition+=increment;
            rightDown = gamepad1.dpad_right;

            if(gamepad1.a && !aDown) {
                intakeClawPivotPosition-=(increment/10);
            }
            aDown = gamepad1.a;
            if(gamepad1.b && !bDown) {
                intakeClawPivotPosition+=(increment/10);
            }
            bDown = gamepad1.b;

            if(gamepad1.x && !xDown) {
                increment-=0.5;
            }
            xDown = gamepad1.x;
            if(gamepad1.y && !yDown) {
                increment+=0.5;
            }
            yDown = gamepad1.y;

            telemetry.addData("Increment", increment);
            telemetry.addData("X Position", xPosition);
            telemetry.addData("Y Position", intakeSlidePosition);
            telemetry.addData("Rotation", intakeClawPivotPosition);
            telemetry.update();
            exit = gamepad1.share;
        }

        telemetry.addLine("Finalized");
        telemetry.addData("X Position", xPosition);
        telemetry.addData("Y Position", intakeSlidePosition);
        telemetry.addData("Rotation", intakeClawPivotPosition);
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.intakePivot.flipBack();
        robot.outtakePivot.flipFront();
        double horizontalSlidePosition = 0;
        robot.intakeClaw.openTo(INTAKE_CLAW_OPEN_POSITION);
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.5);



        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        State curState = State.idle;

        //IMPLEMENT THESE TRAJECTORY SEQUENCES ASYNCHRONOUSLY TO FOLLOW AUTO PATH
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-14, 11.5, 1.4318))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-20.8, 11.27, 1.5443))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-22.2, 14.47, 1.9))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-10.5, 60, 0.0224))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(getSampleFiveTrajectory())
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-10.5, 60, 0.0224))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(-14, 11.5, Math.PI/2))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(traj1);
        curState = State.traj1;
        boolean raiseVertSlides = true;
        boolean lowerVertSlides = false;

        boolean[] oneTimeSwitch = new boolean[100];
        for (int i = 0; i < 100; i++) {
            oneTimeSwitch[i] = true;
        }

        timer.reset();
        while (opModeIsActive()) {
            switch (curState) {
                case idle:
                    break;
                case traj1:
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.2) {
                        oneTimeSwitch[0] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj2);
                        curState = State.traj2;
                        timer.reset();
                    }
                    break;
                case traj2:
                    if (oneTimeSwitch[1] && timer.seconds() > 0.5) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.625;
                        oneTimeSwitch[1] = false;
                    }
                    if (timer.seconds() > 0.8) {
                        intakeGrab();
                    }
                    if (oneTimeSwitch[2] && timer.seconds() > 1.4) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[2] = false;
                    }
                    if(timer.seconds()>1.5&&timer.seconds()<2.4) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.4) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.5) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 2.8) {
                        drive.followTrajectoryAsync(traj3);
                        curState = State.traj3;
                        timer.reset();
                    }
                    break;
                case traj3:
                    if (oneTimeSwitch[3]) {
                        raiseVertSlides = true;
                        oneTimeSwitch[3] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.2) {
                        oneTimeSwitch[4] = false;
                        intakeFlipOut();
                        horizontalSlidePosition = 0.5;
                        drive.followTrajectoryAsync(traj4);
                        curState = State.traj4;
                        timer.reset();
                    }

                    break;

                case traj4:
                    if (oneTimeSwitch[5] && timer.seconds() > 0.8) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.685;
                        oneTimeSwitch[5] = false;
                    }

                    if (timer.seconds() > 1.1 && timer.seconds() < 2) {
                        intakeGrab();
                    }

                    if (oneTimeSwitch[6] && timer.seconds() > 2) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[6] = false;
                    }
                    if(timer.seconds()>1.9&&timer.seconds()<2.6) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.9) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 3) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 3.25) {
                        drive.followTrajectoryAsync(traj5);
                        curState = State.traj5;
                        timer.reset();
                    }
                    break;
                case traj5:
                    if (oneTimeSwitch[7]) {
                        raiseVertSlides = true;
                        oneTimeSwitch[7] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.2) {
                        intakeFlipOut();
                        intakeEdgeClawPosition();
                        horizontalSlidePosition = 0.2;
                        drive.followTrajectoryAsync(traj6);
                        curState = State.traj6;
                        timer.reset();
                    }
                    break;
                case traj6:
                    if (oneTimeSwitch[9] && timer.seconds() > 0.8 && timer.seconds()<3.7) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.6;
                        robot.clawPivot.flipTo(0.5);
                        oneTimeSwitch[9] = false;
                    }

                    if(timer.seconds() > 0.8 && timer.seconds() < 1.3) {
                        intakeEdgeClawPosition();
                    }

                    if(timer.seconds() > 0.8 && timer.seconds()<1.5) {
                        horizontalSlidePosition = 0.625;
                    }

                    if (timer.seconds() > 1.3 && oneTimeSwitch[12]) {
                        oneTimeSwitch[12] = false;
                        intakeGrab();
                    }

                    if (oneTimeSwitch[13] && timer.seconds() > 1.8) {
                        oneTimeSwitch[13] = false;
                        horizontalSlidePosition = 0;
                        robot.clawPivot.flipTo(0.5);
                    }
                    if(timer.seconds()>1.8&&timer.seconds()<3.3) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.7) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.8) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 3.1) {
                        drive.followTrajectoryAsync(traj7);
                        curState = State.traj7;
                        timer.reset();
                    }

                    break;
                case traj7:
                    if (oneTimeSwitch[14]) {
                        raiseVertSlides = true;
                        oneTimeSwitch[14] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.2) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(traj8);
                        curState = State.traj8;
                        timer.reset();
                    }
                    break;
                case traj8:
                    if(!drive.isBusy()) {
                        robot.intakePivot.underBar();
                        horizontalSlidePosition = getIntakeSlidePosition();
                        drive.followTrajectoryAsync(traj9);
                        curState = State.traj9;
                        timer.reset();
                    }
                    break;
                case traj9:
                    if(timer.seconds() > 0 && timer.seconds() < 0.5) {
                        robot.clawPivot.flipTo(getIntakeClawPivotPosition());
                    }
                    if (timer.seconds() > 1 && timer.seconds() < 1.5) {
                        intakeFlipOut();
                    }
                    if (timer.seconds() > 1.5 && timer.seconds() < 1.8) {
                        intakeGrab();
                    }
                    if (timer.seconds() > 2 && timer.seconds() < 3) {
                        horizontalSlidePosition = 0;
                        robot.intakePivot.underBar();
                    }
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        robot.clawPivot.flipTo(0.5);
                        drive.followTrajectoryAsync(traj10);
                        curState = State.traj10;
                        timer.reset();
                    }
                    break;
                case traj10:
                    if(!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj11);
                        curState = State.traj11;
                        timer.reset();
                    }
                case traj11:
                    if (timer.seconds() > 0.1 && timer.seconds() < 0.8) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 0.9) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 1) {
                        intakeLetGo();
                    }
                    if (oneTimeSwitch[19] && timer.seconds() > 1.2) {
                        raiseVertSlides = true;
                        oneTimeSwitch[19] = false;
                    }
                    scoreFinalBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 3.2) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(traj12);
                        curState = State.traj12;
                        timer.reset();
                    }
                    break;
                case traj12:
                    if (!drive.isBusy()) {
                        curState = State.idle;
                        timer.reset();
                    }
                    break;
            }

            robot.horizontalSlide.goTo(horizontalSlidePosition);

            if (raiseVertSlides) {
                robot.verticalSlide.extendFull();
            } else {
                robot.verticalSlide.retractFull();
            }


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("raiseVertSlides", raiseVertSlides);
            telemetry.addData("lowerVertSlides", lowerVertSlides);
            telemetry.addData("Current Time", timer.seconds());
            telemetry.addData("rightVertical", robot.verticalSlide.getLeftPosition());
            telemetry.addData("leftVertical", robot.verticalSlide.getRightPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }

    void scoreBasket(ElapsedTime timer) {
        if (timer.seconds() > 1.5) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 2) {
            outtakeLetGo();
        }
        if (timer.seconds() > 2.2) {
            outtakeFlipIn();
        }
    }

    void scoreFinalBasket(ElapsedTime timer) {
        if (timer.seconds() > 2.3) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 2.8) {
            outtakeLetGo();
        }
        if (timer.seconds() > 3) {
            outtakeFlipIn();
        }
    }

    void outtakeFlipOut() {
        robot.outtakePivot.flipTo(0.45);
    }


    void outtakeFlipIn() {
        robot.outtakePivot.flipFront();
    }

    void outtakeLetGo() {
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_OPEN_POSITION);
    }

    void outtakeGrab() {
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
    }

    void intakeLetGo() {
        robot.intakeClaw.openTo(0.25);
    }

    void intakeGrab() {
        robot.intakeClaw.close();
    }

    void intakeEdgeClawPosition() {
        robot.intakeClaw.openTo(0.35);
    }

    void intakeFlipOut() {
        robot.intakePivot.flipFront();
    }

    void intakeFlipIn() {
        robot.intakePivot.flipBack();
    }

}