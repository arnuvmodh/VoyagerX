package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class SixSampleSolo extends LinearOpMode {

    enum State {
        presetOuttake,
        firstSampleIntake,
        firstSampleOuttake,
        secondSampleIntake,
        secondSampleOuttake,
        thirdSampleIntake,
        thirdSampleOuttake,
        firstSubmersibleIntake,
        firstSubmersibleOuttake,
        secondSubmersibleIntake,
        secondSubmersibleOuttake,
        idle
    }

    Robot robot;
    private SampleMecanumDrive drive;

    double horizontalSlidePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        SubmersibleUI firstSubmersibleSample = new SubmersibleUI(0);
        SubmersibleUI secondSubmersibleSample = new SubmersibleUI(1);

        String allianceColor = "none";
        String oppositeAllianceColor = "none";
        while(!isStopRequested()) {
            telemetry.addLine("X - Blue");
            telemetry.addLine("O - Red");
            telemetry.update();
            if(gamepad1.a) {
                allianceColor = "blue";
                oppositeAllianceColor = "red";
                break;
            }
            if(gamepad1.b) {
                allianceColor = "red";
                oppositeAllianceColor = "blue";
                break;
            }
        }
        while(!isStopRequested()) {
            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("Opposite Alliance Color", oppositeAllianceColor);
            telemetry.addLine("Press Share to Continue");
            telemetry.update();
            if(gamepad1.share) break;
        }
        while(!isStopRequested()) {
            if(firstSubmersibleSample.userInput(gamepad1, telemetry)) break;
        }
        while(!isStopRequested()) {
            telemetry.addLine("Press Share to Continue");
            telemetry.update();
            if(gamepad1.share) break;
        }
        while(!isStopRequested()) {
            if(secondSubmersibleSample.userInput(gamepad1, telemetry)) break;
        }
        while(!isStopRequested()) {
            telemetry.addLine("Press Share to Continue");
            telemetry.update();
            if(gamepad1.share) break;
        }

        telemetry.addLine("Selection Finalized");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.intakePivot.flipTo(0.25);
        robot.outtakePivot.flipFront();
        robot.outtakeClaw.openTo(0.9);
        robot.clawPivot.flipTo(0.5);
        robot.hangPivot.setPosition(0.5);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        State curState = State.idle;

        Trajectory presetOuttake = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory firstSampleIntake = drive.trajectoryBuilder(presetOuttake.end())
                .lineToSplineHeading(new Pose2d(-10.7527, 14.1049, 1.5431))
                .build();
        Trajectory firstSampleOuttake = drive.trajectoryBuilder(firstSampleIntake.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory secondSampleIntake = drive.trajectoryBuilder(firstSampleOuttake.end())
                .lineToSplineHeading(new Pose2d(-19.5, 13.9176, 1.5766))
                .build();
        Trajectory secondSampleOuttake = drive.trajectoryBuilder(secondSampleIntake.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory thirdSampleIntake = drive.trajectoryBuilder(secondSampleOuttake.end())
                .lineToSplineHeading(new Pose2d(-22.5, 16.8352, 1.8539))
                .build();
        Trajectory thirdSampleOuttake = drive.trajectoryBuilder(thirdSampleIntake.end())
                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                .build();
        Trajectory firstSubmersibleIntake = drive.trajectoryBuilder(thirdSampleOuttake.end())
                .splineToLinearHeading(firstSubmersibleSample.getTrajectory(),0)
                .build();
        Trajectory firstSubmersibleOuttake = drive.trajectoryBuilder(firstSubmersibleIntake.end(), true)
                .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-180))
                .build();
        Trajectory secondSubmersibleIntake = drive.trajectoryBuilder(firstSubmersibleOuttake.end())
                .splineToLinearHeading(secondSubmersibleSample.getTrajectory(),0)
                .build();
        Trajectory secondSubmersibleOuttake = drive.trajectoryBuilder(secondSubmersibleIntake.end(), true)
                .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-180))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(presetOuttake);
        curState = State.presetOuttake;
        boolean raiseVertSlides = true;

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
            switch (curState) {
                case idle:
                    break;
                case presetOuttake:
                    if(timer.seconds() > 0 && timer.seconds() < 1.2) {
                        outtakeFlipUp();
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.5) {
                        oneTimeSwitch[0] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(firstSampleIntake);
                        curState = State.firstSampleIntake;
                        timer.reset();
                    }
                    break;
                case firstSampleIntake:
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
                        firstSampleOuttake = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                                .build();
                        drive.followTrajectoryAsync(firstSampleOuttake);
                        curState = State.firstSampleOuttake;
                        timer.reset();
                    }
                    break;
                case firstSampleOuttake:
                    if (oneTimeSwitch[3]) {
                        outtakeFlipUp();
                        raiseVertSlides = true;
                        oneTimeSwitch[3] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.5) {
                        oneTimeSwitch[4] = false;
                        intakeFlipOut();
                        horizontalSlidePosition = 0.1;
                        drive.followTrajectoryAsync(secondSampleIntake);
                        curState = State.secondSampleIntake;
                        timer.reset();
                    }
                    break;
                case secondSampleIntake:
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
                        secondSampleOuttake = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                                .build();
                        drive.followTrajectoryAsync(secondSampleOuttake);
                        curState = State.secondSampleOuttake;
                        timer.reset();
                    }
                    break;
                case secondSampleOuttake:
                    if (oneTimeSwitch[7]) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[7] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.5) {
                        intakeFlipOut();
                        horizontalSlidePosition = 0.2;
                        drive.followTrajectoryAsync(thirdSampleIntake);
                        curState = State.thirdSampleIntake;
                        timer.reset();
                    }
                    break;
                case thirdSampleIntake:
                    if (oneTimeSwitch[9] && timer.seconds() > 0.8 && timer.seconds()<3.7) {
                        raiseVertSlides = false;
                        horizontalSlidePosition = 0.2;
                        robot.clawPivot.flipTo(0.475);
                        robot.spintake.spinIn(1);
                        oneTimeSwitch[9] = false;
                    }

                    if(timer.seconds() > 1 && timer.seconds()<1.7) {
                        horizontalSlidePosition = 1;
                        intakeGrab();
                    }

                    if (oneTimeSwitch[13] && timer.seconds() > 1.7) {
                        oneTimeSwitch[13] = false;
                        horizontalSlidePosition = 0;
                        robot.clawPivot.flipTo(0.5);
                    }
                    if(timer.seconds()>1.7&&timer.seconds()<2.2) {
                        intakeFlipIn();
                    }
                    if (timer.seconds() > 2.2 && timer.seconds() < 2.4) {
                        outtakeGrab();
                    }
                    if (timer.seconds() > 2.4 && timer.seconds() < 2.6) {
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 2.6) {
                        thirdSampleOuttake = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-18.25, 6.2, 0.75))
                                .build();
                        drive.followTrajectoryAsync(thirdSampleOuttake);
                        curState = State.thirdSampleOuttake;
                        timer.reset();
                    }
                    break;
                case thirdSampleOuttake:
                    if (oneTimeSwitch[14]) {
                        raiseVertSlides = true;
                        outtakeFlipUp();
                        oneTimeSwitch[14] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 1.5) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(firstSubmersibleIntake);
                        curState = State.firstSubmersibleIntake;
                        timer.reset();
                    }
                    break;
                case firstSubmersibleIntake:
                    if(timer.seconds() > 2 && !fifthPositionReached) {
                        timer.reset();
                        fifthPositionReached = true;
                    }
                    if(fifthPositionReached) {
                        if(!fifthIntakeSuccess) {
                            if(robot.colorSensor.getColorAsString().equals(oppositeAllianceColor)) {
                                robot.spintake.spinOut(1);
                                break;
                            }
                            if (timer.seconds() > 0 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0);
                                horizontalSlidePosition = firstSubmersibleSample.getIntakeSlidePosition();
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

                            if((robot.colorSensor.getColorAsString().equals("yellow") || robot.colorSensor.getColorAsString().equals(allianceColor))) {
                                intakeFlipIn();
                                fifthIntakeSuccess=true;
                            }

                            if(timer.seconds() > 2.5 && timer.seconds() < 3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                if(currentPose.getY() + secondSubmersibleSample.getFailStrafe() > 74 ||
                                        currentPose.getY() + secondSubmersibleSample.getFailStrafe() < 52
                                ) secondSubmersibleSample.reverseFailStrafe();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX(), currentPose.getY() + firstSubmersibleSample.getFailStrafe(), currentPose.getHeading());
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
                                firstSubmersibleOuttake = drive.trajectoryBuilder(firstSubmersibleIntake.end(), true)
                                        .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-180))
                                        .build();
                                drive.followTrajectoryAsync(firstSubmersibleOuttake);
                                curState = State.firstSubmersibleOuttake;
                                timer.reset();
                            }
                        }
                    }
                    break;
                case firstSubmersibleOuttake:
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
                    if (!drive.isBusy() && timer.seconds() > 2.7) {
                        raiseVertSlides = false;
                        drive.followTrajectoryAsync(secondSubmersibleIntake);
                        curState = State.secondSubmersibleIntake;
                        timer.reset();
                    }
                    break;
                case secondSubmersibleIntake:
                    if(timer.seconds() > 2 && !sixthPositionReached) {
                        timer.reset();
                        sixthPositionReached = true;
                    }
                    if(sixthPositionReached) {
                        if(!sixthIntakeSuccess) {
                            if(robot.colorSensor.getColorAsString().equals(oppositeAllianceColor)) {
                                robot.spintake.spinOut(1);
                                break;
                            }
                            if (timer.seconds() > 0 && timer.seconds() < 0.5) {
                                robot.hangPivot.setPosition(0);
                                horizontalSlidePosition = secondSubmersibleSample.getIntakeSlidePosition();
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

                            if((robot.colorSensor.getColorAsString().equals("yellow") || robot.colorSensor.getColorAsString().equals(allianceColor))) {
                                intakeFlipIn();
                                sixthIntakeSuccess=true;
                            }
                            if(timer.seconds() > 2.5 && timer.seconds() < 3) {
                                Pose2d currentPose = drive.getPoseEstimate();
                                if(currentPose.getY() + secondSubmersibleSample.getFailStrafe() > 74 ||
                                        currentPose.getY() + secondSubmersibleSample.getFailStrafe() < 52
                                ) secondSubmersibleSample.reverseFailStrafe();
                                Pose2d adjustedPose = new Pose2d(currentPose.getX(), currentPose.getY() + secondSubmersibleSample.getFailStrafe(), currentPose.getHeading());
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
                                secondSubmersibleOuttake = drive.trajectoryBuilder(secondSubmersibleIntake.end(), true)
                                        .splineToLinearHeading(new Pose2d(-18.25, 6.2, 0.75), Math.toRadians(-180))
                                        .build();
                                drive.followTrajectoryAsync(secondSubmersibleOuttake);
                                curState = State.secondSubmersibleOuttake;
                                timer.reset();
                            }
                        }
                    }
                    break;
                case secondSubmersibleOuttake:
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
                    if (!drive.isBusy() && timer.seconds() > 2.7) {
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
        if (timer.seconds() > 1.2 && timer.seconds() < 1.35) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 1.35) {
            outtakeLetGo();
        }
        if (timer.seconds() > 1.5) {
            outtakeFlipIn();
        }
    }

    void scoreSubmersibleBasket(ElapsedTime timer) {
        if (timer.seconds() > 2.2 && timer.seconds() < 2.35) {
            outtakeFlipOut();
        }
        if (timer.seconds() > 2.35) {
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
        robot.outtakeClaw.openTo(0.3);
    }

    void outtakeGrab() {
        robot.outtakeClaw.openTo(0.9);
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