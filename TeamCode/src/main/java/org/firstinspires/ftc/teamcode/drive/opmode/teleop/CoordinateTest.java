package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@TeleOp
public class CoordinateTest extends LinearOpMode {

    private Robot robot;
    private SampleMecanumDrive drive;
    private double xPos = 0;
    private double slidePos = 0;
    private boolean leftDDown = false, rightDDown = false;
    private boolean upDDown = false, downDDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upDDown) {
                slidePos += 0.1;
            } else if (gamepad1.dpad_down && !downDDown) {
                slidePos -= 0.1;
            }
            upDDown = gamepad1.dpad_up;
            downDDown = gamepad1.dpad_down;

            robot.horizontalSlide.goTo(slidePos);
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

    private void updateTelemetry() {

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("X", poseEstimate.getX());
        telemetry.addData("Y", poseEstimate.getY());
        telemetry.addData("Heading", poseEstimate.getHeading());
        telemetry.addData("Slide", slidePos);
        telemetry.update();
    }
}
