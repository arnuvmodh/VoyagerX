package org.firstinspires.ftc.teamcode.drive.opmode.auto.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.time.FiveSampleTime;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake.OuttakePivot;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@Autonomous()
public class FiveSample extends LinearOpMode {

    public Trajectory presetOuttake, firstIntake, firstOuttake, secondIntake, secondOuttake, thirdIntake, thirdOuttake, fourthIntake, fourthOuttake;
    enum DriveState {
        IDLE,
        PRESET_OUTTAKE,
        FIRST_INTAKE,
        FIRST_OUTTAKE,
        SECOND_INTAKE,
        SECOND_OUTTAKE,
        THIRD_INTAKE,
        THIRD_OUTTAKE,
        FOURTH_INTAKE,
        FOURTH_OUTTAKE
    }
    enum IntakeState {
        IDLE,
        EXTEND,
        GRAB,
        RETRACT,
        TRANSFER,
        RELEASE
    }
    enum OuttakeState {
        IDLE,
        FLIP_BACK,
        RELEASE,
        FLIP_FRONT
    }

    private DriveState driveState = DriveState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    private OuttakeState outtakeState = OuttakeState.IDLE;

    private final Pose2d highBucketPosition = new Pose2d(-18.25, 5.2, 0.75);
    private final Pose2d firstIntakePosition = new Pose2d(-14, 10, 1.4318);
    private final Pose2d secondIntakePosition = new Pose2d(-20.8, 10.27, 1.5443);
    private final Pose2d thirdIntakePosition = new Pose2d(-22.2, 13.47, 1.9);
    private final Pose2d fourthIntakePosition = new Pose2d(-13.2755, 3.4075, 0.013);

    public void buildTrajectories() {
        presetOuttake = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(highBucketPosition)
                .build();
        firstIntake = drive.trajectoryBuilder(presetOuttake.end())
                .lineToSplineHeading(firstIntakePosition)
                .build();
        firstOuttake = drive.trajectoryBuilder(firstIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        secondIntake = drive.trajectoryBuilder(firstOuttake.end())
                .lineToSplineHeading(secondIntakePosition)
                .build();
        secondOuttake = drive.trajectoryBuilder(secondIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        thirdIntake = drive.trajectoryBuilder(secondOuttake.end())
                .lineToSplineHeading(thirdIntakePosition)
                .build();
        thirdOuttake = drive.trajectoryBuilder(thirdIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
        fourthIntake = drive.trajectoryBuilder(thirdOuttake.end())
                .lineToSplineHeading(fourthIntakePosition)
                .build();
        fourthOuttake = drive.trajectoryBuilder(fourthIntake.end())
                .lineToSplineHeading(highBucketPosition)
                .build();
    }

    Robot robot;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        buildTrajectories();

        robot.outtakePivot.flipFront();
        robot.outtakeClaw.openTo(1);

        waitForStart();
        if (isStopRequested()) return;
//        drive.followTrajectoryAsync(presetOuttake);
        driveState = DriveState.PRESET_OUTTAKE;

        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            handleOuttake();
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("Drive State", driveState);
            telemetry.addData("Outtake State", outtakeState);
            telemetry.addData("Preset Outtake Start", presetOuttake.start());
            telemetry.addData("Preset Outtake End", presetOuttake.end());
            telemetry.addData("Drive Busy", drive.isBusy());
            telemetry.addData("Left Outtake Claw", robot.outtakeClaw.getLeftPosition());
            telemetry.addData("Right Outtake Claw", robot.outtakeClaw.getRightPosition());
            telemetry.addData("Left Outtake Pivot", robot.outtakePivot.getLeftPosition());
            telemetry.addData("Right Outtake Pivot", robot.outtakePivot.getRightPosition());

            telemetry.update();
        }
    }

    private boolean locked = false;
    private boolean stateHandled = false;
    private void handleDrive() {
        switch(driveState) {
            case IDLE:
                break;
            case PRESET_OUTTAKE:
                if(!drive.isBusy()) {
                    if(!stateHandled) {
                        outtakeState = OuttakeState.FLIP_BACK;
                        stateHandled = true;
                    }
                    if(!locked) {
                        stateHandled = false;
                        driveState = DriveState.FIRST_INTAKE;
//                        drive.followTrajectoryAsync(firstIntake);
                    }
                }
                break;
        }
    }

    private void handleIntake() {

    }

    private void handleOuttake() {
        switch(outtakeState) {
            case IDLE:
                break;
            case FLIP_BACK:
                locked = true;
                robot.outtakePivot.flipSampleScore();
                if(robot.outtakePivot.isAt(OuttakePivot.SAMPLE_SCORE_POSITION)) outtakeState = OuttakeState.RELEASE;
                break;
            case RELEASE:
                locked = true;
                robot.outtakeClaw.openTo(0.3);
                if(robot.outtakeClaw.isAt(0.3)) outtakeState = OuttakeState.FLIP_FRONT;
                break;
            case FLIP_FRONT:
                locked = false;
                robot.outtakePivot.flipFront();
                if(robot.outtakePivot.isIn()) outtakeState = OuttakeState.IDLE;
                break;
        }
    }
}