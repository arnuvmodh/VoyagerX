package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@TeleOp
public class TwoDriver extends LinearOpMode{
    private Robot robot;
    private SampleMecanumDrive drive;

    private int verticalSlidePosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializePositions();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            controlDrivetrain();
            controlVerticalSlides();
            robot.verticalSlide.goTo(verticalSlidePosition, 1);
            drive.update();
            updateTelemetry();
        }
    }

    private void initialize() {
        robot = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    private void initializePositions() {
        robot.verticalSlide.retractFull();
        robot.horizontalSlide.retractFull();
        robot.outtakePivot.flipFront();
        robot.intakePivot.flipBack();
        robot.outtakeClaw.release();
        robot.intakeClaw.close();
    }

    private void controlDrivetrain() {
        if(gamepad1.a) {
            initialize();
        }
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
    }

    private void controlVerticalSlides() {
        if(gamepad2.a) {
            verticalSlidePosition=0;
        }
        if(gamepad2.b) {
            verticalSlidePosition=1950;
        }
    }

    private void updateTelemetry() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
