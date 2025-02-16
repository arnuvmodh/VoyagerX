package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class SixSampleSolo extends LinearOpMode {
    final double OUTTAKE_CLAW_OPEN_POSITION = 0.3;
    final double OUTTAKE_CLAW_CLOSE_POSITION = 0.9;
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
        idle
    }

    Robot robot;
    private SampleMecanumDrive drive;
    double horizontalSlidePosition = 0;

    double fiveXPosition = 0;
    double fiveIntakeSlidePosition = 0;
    double fiveFailStrafe = 5;

    Pose2d getSampleFiveTrajectory() {
        return new Pose2d(17.5, 60+fiveXPosition, 6.2498);
    }

    double getFiveIntakeSlidePosition() {
        return fiveIntakeSlidePosition;
    }

    double sixXPosition = 0;
    double sixIntakeSlidePosition = 0;
    double sixFailStrafe = 5;

    Pose2d getSampleSixTrajectory() {
        return new Pose2d(17.5, 60+sixXPosition, 6.2498);
    }

    double getSixIntakeSlidePosition() {
        return sixIntakeSlidePosition;
    }
    double increment = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        boolean exit = false;
        boolean upDown = false, downDown = false, leftDown = false, rightDown = false;
        boolean aDown = false, bDown = false, xDown = false, yDown = false;
        int currentSample = 5;
        while(!exit && !isStopRequested()) {
            if(currentSample == 5) {
                if(gamepad1.dpad_up && !upDown) fiveIntakeSlidePosition+=(increment/10);
                upDown = gamepad1.dpad_up;
                if(gamepad1.dpad_down && !downDown) fiveIntakeSlidePosition-=(increment/10);
                downDown = gamepad1.dpad_down;
                if(gamepad1.dpad_left && !leftDown) fiveXPosition-=increment;
                leftDown = gamepad1.dpad_left;
                if(gamepad1.dpad_right && !rightDown) fiveXPosition+=increment;
                rightDown = gamepad1.dpad_right;

                if(gamepad1.b && !bDown) {
                    fiveFailStrafe = -1*fiveFailStrafe;
                }
                bDown = gamepad1.b;
            }
            else if(currentSample == 6) {
                if(gamepad1.dpad_up && !upDown) sixIntakeSlidePosition+=(increment/10);
                upDown = gamepad1.dpad_up;
                if(gamepad1.dpad_down && !downDown) sixIntakeSlidePosition-=(increment/10);
                downDown = gamepad1.dpad_down;
                if(gamepad1.dpad_left && !leftDown) sixXPosition-=increment;
                leftDown = gamepad1.dpad_left;
                if(gamepad1.dpad_right && !rightDown) sixXPosition+=increment;
                rightDown = gamepad1.dpad_right;

                if(gamepad1.b && !bDown) {
                    sixFailStrafe = -1*sixFailStrafe;
                }
                bDown = gamepad1.b;
            }

            if(gamepad1.a && !aDown) {
                currentSample+=1;
            }
            aDown = gamepad1.a;

            if(gamepad1.x && !xDown) {
                increment-=0.5;
            }
            xDown = gamepad1.x;
            if(gamepad1.y && !yDown) {
                increment+=0.5;
            }
            yDown = gamepad1.y;

            if(currentSample==5) {
                telemetry.addLine("Fifth Sample");
                telemetry.addData("Increment", increment);
                telemetry.addData("X Position", fiveXPosition);
                telemetry.addData("Intake Slide Position", fiveIntakeSlidePosition);
                telemetry.addData("Fail Strafe", fiveFailStrafe);
                telemetry.update();
            }
            else if(currentSample==6) {
                telemetry.addLine("Sixth Sample");
                telemetry.addData("Increment", increment);
                telemetry.addData("X Position", sixXPosition);
                telemetry.addData("Intake Slide Position", sixIntakeSlidePosition);
                telemetry.addData("Fail Strafe", sixFailStrafe);
                telemetry.update();
            }

            exit = gamepad1.share;
        }

        telemetry.addLine("Finalized");
        telemetry.addData("Increment", increment);
        telemetry.addData("X Position", fiveXPosition);
        telemetry.addData("Intake Slide Position", fiveIntakeSlidePosition);
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.intakePivot.flipTo(0.25);
        robot.outtakePivot.flipFront();
        robot.outtakeClaw.openTo(OUTTAKE_CLAW_CLOSE_POSITION);
        robot.clawPivot.flipTo(0.5);
        robot.hangPivot.setPosition(0.5);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        State curState = State.idle;

        //IMPLEMENT THESE TRAJECTORY SEQUENCES ASYNCHRONOUSLY TO FOLLOW AUTO PATH
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-10.7527, 14.1049, 1.5431))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-19, 13.9176, 1.5766))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-22.5, 16.8352, 1.8539))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .splineToLinearHeading(getSampleFiveTrajectory(),0)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), true)
                .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-90))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .splineToLinearHeading(getSampleSixTrajectory(),0)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end(), true)
                .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(traj1);
        curState = State.traj1;
        boolean raiseVertSlides = true;
        boolean lowerVertSlides = false;

        boolean fifthIntakeSuccess = false;
        boolean fifthPositionReached = false;
        boolean sixthIntakeSuccess = false;
        boolean sixthPositionReached = false;
        boolean[] oneTimeSwitch = new boolean[100];
        for (int i = 0; i < 100; i++) {
            oneTimeSwitch[i] = true;
        }

        timer.reset();
        while (opModeIsActive()) {
            if(oneTimeSwitch[99]) {
                runtime.reset();
                oneTimeSwitch[99] = false;
            }
            if(runtime.seconds() >= 29.8 && curState!=State.idle) {
                robot.outtakePivot.flipTo(0.6);
                if(runtime.seconds() >= 29.9) {
                    robot.outtakeClaw.open();
                }
            }
            switch (curState) {
                case idle:
                    break;
                case traj1:
                    if(timer.seconds() > 0 && timer.seconds() < 1.5) {
                        outtakeFlipUp();
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.8) {
                        oneTimeSwitch[0] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj2);
                        curState = State.traj2;
                        timer.reset();
                    }
                    break;
                case traj2:
                    if (oneTimeSwitch[1] && timer.seconds() > 0.8) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.285;
                        robot.spintake.spinIn(1);
                        oneTimeSwitch[1] = false;
                    }
                    if (timer.seconds() > 1 && timer.seconds() < 1.5) {
                        horizontalSlidePosition=1;
                        intakeGrab();
                    }
                    if (oneTimeSwitch[2] && timer.seconds() > 1.5) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[2] = false;
                    }
                    if(timer.seconds()>1.6&&timer.seconds()<2.1) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.1 && timer.seconds() < 2.3) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.3 && timer.seconds() < 2.5) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 2.5) {
                        drive.followTrajectoryAsync(traj3);
                        curState = State.traj3;
                        timer.reset();
                    }
                    break;
                case traj3:
                    if (oneTimeSwitch[3]) {
                        outtakeFlipUp();
                        raiseVertSlides = true;
                        oneTimeSwitch[3] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.8) {
                        oneTimeSwitch[4] = false;
                        intakeFlipOut();
                        horizontalSlidePosition = 0.1;
                        drive.followTrajectoryAsync(traj4);
                        curState = State.traj4;
                        timer.reset();
                    }
                    break;
                case traj4:
                    if (oneTimeSwitch[5] && timer.seconds() > 0.8) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.1;
                        robot.spintake.spinIn(1);
                        oneTimeSwitch[5] = false;
                    }

                    if (timer.seconds() > 1 && timer.seconds() < 1.5) {
                        horizontalSlidePosition=1;
                        intakeGrab();
                    }

                    if (oneTimeSwitch[6] && timer.seconds() > 1.5) {
                        horizontalSlidePosition = 0;
                        oneTimeSwitch[6] = false;
                    }
                    if(timer.seconds()>1.6&&timer.seconds()<2.1) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.1 && timer.seconds() < 2.3) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.3 && timer.seconds() < 2.5) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 2.5) {
                        drive.followTrajectoryAsync(traj5);
                        curState = State.traj5;
                        timer.reset();
                    }
                    break;
                case traj5:
                    if (oneTimeSwitch[7]) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[7] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.8) {
                        intakeFlipOut();
                        horizontalSlidePosition = 0.2;
                        drive.followTrajectoryAsync(traj6);
                        curState = State.traj6;
                        timer.reset();
                    }
                    break;
                case traj6:
                    if (oneTimeSwitch[9] && timer.seconds() > 0.8 && timer.seconds()<3.7) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.2;
                        robot.clawPivot.flipTo(0.475);
                        robot.spintake.spinIn(1);
                        oneTimeSwitch[9] = false;
                    }

                    if(timer.seconds() > 1 && timer.seconds()<1.5) {
                        horizontalSlidePosition = 1;
                        intakeGrab();
                    }

                    if (oneTimeSwitch[13] && timer.seconds() > 1.5) {
                        oneTimeSwitch[13] = false;
                        horizontalSlidePosition = 0;
                        robot.clawPivot.flipTo(0.5);
                    }
                    if(timer.seconds()>1.5&&timer.seconds()<2.2) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.2 && timer.seconds() < 2.4) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.4 && timer.seconds() < 2.6) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 2.6) {
                        drive.followTrajectoryAsync(traj7);
                        curState = State.traj7;
                        timer.reset();
                    }
                    break;
                case traj7:
                    if (oneTimeSwitch[14]) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[14] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.8) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(traj8);
                        curState = State.traj8;
                        timer.reset();
                    }
                    break;
                case traj8:
                    if(timer.seconds() > 2 && !fifthPositionReached) {
                        timer.reset();
                        fifthPositionReached = true;
                    }
                    if(fifthPositionReached) {
                        if(!fifthIntakeSuccess) {
                            if(robot.colorSensor.getColor().equals("Blue")) {
                                robot.spintake.spinOut(1);
                                break;
                            }
                            if (timer.seconds() > 0 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0);
                                horizontalSlidePosition = getFiveIntakeSlidePosition();
                            }

                            if(timer.seconds() >= 0.25 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0.5);
                            }

                            if(timer.seconds() > 0.35 && timer.seconds() < 0.5) {
                                intakeFlipSubmersible();
                            }

                            if(timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX()-1, currentPose.getY(), currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                            }

                            if(timer.seconds() > 1 && timer.seconds() < 1.3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX()+1, currentPose.getY(), currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                            }

                            if (timer.seconds() > 1 && timer.seconds() < 2) {
                                robot.clawPivot.flipTo(0.5);
                                horizontalSlidePosition = 1;
                                intakeGrab();
                            }

                            if((robot.colorSensor.getColor().equals("Yellow") || robot.colorSensor.getColor().equals("Red"))) {
                                intakeFlipIn();
                                fifthIntakeSuccess=true;
                            }

                            if(timer.seconds() > 2.5 && timer.seconds() < 3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX(), currentPose.getY() + fiveFailStrafe, currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                                intakeFlipIn();
                                horizontalSlidePosition=0;
                                robot.clawPivot.flipTo(0.5);
                            }
                            if(timer.seconds() > 3 && timer.seconds() < 3.5) {
                                timer.reset();
                            }
                        }
                        else {
                            if (!drive.isBusy() && timer.seconds() > 1.6) {
                                intakeFlipIn();
                                horizontalSlidePosition = 0;
                                robot.clawPivot.flipTo(0.5);
                                drive.followTrajectoryAsync(traj9);
                                curState = State.traj9;
                                timer.reset();
                            }
                        }
                    }
                    break;
                case traj9:
                    if (timer.seconds() > 0.5 && timer.seconds() < 0.7) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 0.7 && timer.seconds() < 0.9) {
                        intakeLetGo();
                    }
                    if (oneTimeSwitch[17] && timer.seconds() > 0.9) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[17] = false;
                    }
                    scoreSubmersibleBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.5) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(traj10);
                        curState = State.traj10;
                        timer.reset();
                    }
                    break;
                case traj10:
                    if(timer.seconds() > 2 && !sixthPositionReached) {
                        timer.reset();
                        sixthPositionReached = true;
                    }
                    if(sixthPositionReached) {
                        if(!sixthIntakeSuccess) {
                            if(robot.colorSensor.getColor().equals("Blue")) {
                                robot.spintake.spinOut(1);
                                break;
                            }
                            if (timer.seconds() > 0 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0);
                                horizontalSlidePosition = getFiveIntakeSlidePosition();
                            }

                            if(timer.seconds() >= 0.25 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0.5);
                            }

                            if(timer.seconds() > 0.35 && timer.seconds() < 0.5) {
                                intakeFlipSubmersible();
                            }

                            if(timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX()-1, currentPose.getY(), currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                            }

                            if(timer.seconds() > 1 && timer.seconds() < 1.3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX()+1, currentPose.getY(), currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                            }

                            if (timer.seconds() > 1 && timer.seconds() < 2) {
                                robot.clawPivot.flipTo(0.5);
                                horizontalSlidePosition = 1;
                                intakeGrab();
                            }

                            if((robot.colorSensor.getColor().equals("Yellow") || robot.colorSensor.getColor().equals("Red"))) {
                                intakeFlipIn();
                                sixthIntakeSuccess=true;
                            }
                            if(timer.seconds() > 2.5 && timer.seconds() < 3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX(), currentPose.getY() + sixFailStrafe, currentPose.getHeading());
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(currentPose)
                                        .lineToSplineHeading(adjustedPose)
                                        .build());
                                intakeFlipIn();
                                horizontalSlidePosition=0;
                                robot.clawPivot.flipTo(0.5);
                            }
                            if(timer.seconds() > 3 && timer.seconds() < 3.5) {
                                timer.reset();
                            }
                        }
                        else {
                            if (!drive.isBusy() && timer.seconds() > 1.6) {
                                intakeFlipIn();
                                horizontalSlidePosition = 0;
                                robot.clawPivot.flipTo(0.5);
                                drive.followTrajectoryAsync(traj11);
                                curState = State.traj11;
                                timer.reset();
                            }
                        }
                    }
                    break;
                case traj11:
                    if (timer.seconds() > 0.5 && timer.seconds() < 0.7) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 0.7 && timer.seconds() < 0.9) {
                        intakeLetGo();
                    }
                    if (oneTimeSwitch[19] && timer.seconds() > 0.9) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[19] = false;
                    }
                    scoreSubmersibleBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 2.5) {
                        curState = State.idle;
                        timer.reset();
                        raiseVertSlides = false;
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

            telemetry.addData("Current Time", timer.seconds());
            telemetry.addData("Detected Hue", robot.colorSensor.getHue());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }

    void scoreBasket(ElapsedTime timer) {
        if (timer.seconds() > 1.5 && timer.seconds() < 1.6) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 1.6) {
            outtakeLetGo();
        }
        if (timer.seconds() > 1.8) {
            outtakeFlipIn();
        }
    }

    void scoreSubmersibleBasket(ElapsedTime timer) {
        if (timer.seconds() > 2.2 && timer.seconds() < 2.3) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 2.3) {
            outtakeLetGo();
        }
        if (timer.seconds() > 2.5) {
            outtakeFlipIn();
        }
    }

    void outtakeFlipOut() {
        robot.outtakePivot.flipTo(0.45);
    }

    void outtakeFlipUp() {
        robot.outtakePivot.flipTo(0.7);
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
        horizontalSlidePosition=0.3;
        robot.spintake.spinOut(1);
    }

    void intakeGrab() {
        robot.spintake.spinIn(1);
    }

    void intakeFlipOut() {
        robot.intakePivot.flipTo(0.73);
    }

    void intakeFlipSubmersible() {
        robot.intakePivot.flipTo(0.7325);
    }

    void intakeFlipIn() {
        robot.intakePivot.flipTo(0.245);
    }

    void intakeUnderBar() {
        robot.intakePivot.flipTo(0.655);
    }

}