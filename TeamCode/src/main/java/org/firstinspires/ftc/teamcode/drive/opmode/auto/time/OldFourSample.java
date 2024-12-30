package org.firstinspires.ftc.teamcode.drive.opmode.auto.time;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

@Autonomous()
public class OldFourSample extends LinearOpMode {
    final double INTAKE_CLAW_OPEN_POSITION = 0.1;
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
        idle
    }

    Robot robot;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
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
                .lineToSplineHeading(new Pose2d(-18.25, 5.2, 0.75))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-14, 10, 1.4318))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-18.25, 5.2, 0.75))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-18, 10, 1.6415))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-18.25, 5.2, 0.75))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-22.2, 13.47, 1.9))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-18.25, 5.2, 0.75))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-13.6212, 7.5, (Math.PI / 2)+0.2))
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
                    if (!drive.isBusy() && timer.seconds() > 3) {
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
                        horizontalSlidePosition = 0.55;
                        oneTimeSwitch[1] = false;
                    }
                    if (timer.seconds() > 1.6 && timer.seconds()<3) {
                        intakeGrab();
                    }

                    if (oneTimeSwitch[2] && timer.seconds() > 2.2) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[2] = false;
                    }
                    if(timer.seconds()>2.2&&timer.seconds()<3.3) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 3.3 && timer.seconds() < 4) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 3.8 && timer.seconds()<4.1) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 4.2) {
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
                    if (!drive.isBusy() && timer.seconds() > 3.5) {
                        oneTimeSwitch[4] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj4);
                        curState = State.traj4;
                        timer.reset();
                    }

                    break;
                case traj4:
                    if (oneTimeSwitch[5] && timer.seconds() > 0.8) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.95;
                        oneTimeSwitch[5] = false;
                    }
                    if (timer.seconds() > 1.5 && timer.seconds() < 2.4) {
                        intakeGrab();
                    }

                    if (oneTimeSwitch[6] && timer.seconds() > 2.4) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[6] = false;
                    }
                    if(timer.seconds()>2.4&&timer.seconds()<3.7) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 3.7 && timer.seconds()<4.6) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 4.2 && timer.seconds()<4.6) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 4.6) {
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
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        oneTimeSwitch[8] = false;
                        intakeFlipOut();
                        intakeEdgeClawPosition();
                        drive.followTrajectoryAsync(traj6);
                        curState = State.traj6;
                        timer.reset();
                    }

                    break;
                case traj6:
                    if (oneTimeSwitch[9] && timer.seconds() > 0.8 && timer.seconds()<3.7) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.625;
                        robot.clawPivot.flipTo(0.5);
                        oneTimeSwitch[9] = false;
                    }

                    if(timer.seconds() > 3 && timer.seconds()<3.7) {
                        horizontalSlidePosition = 0.625;
//                        intakeEdgeClawPosition();
                    }

                    if (timer.seconds() > 3.7 && oneTimeSwitch[12]) {
                        oneTimeSwitch[12] = false;
                        intakeGrab();
                    }

                    if (oneTimeSwitch[13] && timer.seconds() > 4) {
                        oneTimeSwitch[13] = false;
                        horizontalSlidePosition = 0;
                        robot.clawPivot.flipTo(0.5);
                    }
                    if(timer.seconds()>4&&timer.seconds()<5.3) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 5.3 && timer.seconds() < 5.8) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 5.8 && timer.seconds() < 6) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 6.2) {
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
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        drive.followTrajectoryAsync(traj8);
                        curState = State.traj8;
                        timer.reset();
                    }
                    break;
                case traj8:
                    if (timer.seconds() > .1) {
                        intakeGrab();
                    }
                    if (oneTimeSwitch[15] && timer.seconds() > 0.2) {
                        raiseVertSlides = false;
                        oneTimeSwitch[15] = false;
                    }
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        curState = State.idle;
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
        if (timer.seconds() > 2.1) {
            outtakeLetGo();
        }
        if (timer.seconds() > 2.3) {
            outtakeFlipIn();
        }
    }

    void outtakeFlipOut() {
        robot.outtakePivot.flipBack();
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
        robot.intakeClaw.open();
    }

    void intakeGrab() {
        robot.intakeClaw.close();
    }

    void intakeEdgeClawPosition() {
        robot.intakeClaw.openTo(0.3);
    }

    void intakeFlipOut() {
        robot.intakePivot.flipFront();
    }

    void intakeFlipIn() {
        robot.intakePivot.flipBack();
    }

}