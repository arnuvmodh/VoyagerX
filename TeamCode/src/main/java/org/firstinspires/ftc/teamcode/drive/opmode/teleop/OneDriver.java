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

@Disabled
public class OneDriver extends LinearOpMode{

    // Drive State
    enum Mode {
        DRIVE,
        INTAKE,
        OUTTAKE,
    }
    private Mode currentMode = Mode.DRIVE;

    // Drive Control Variables
    private double speedReduction = 1;

    // Intake Control Variables
    private double intakeSlidePosition = 0;
    private double intakeClawPosition = 0.3;
    private double intakeArmPivotPosition = 0;
    private double intakeClawPivotPosition = 0.5;

    // Outtake Control Variables
    private int outtakeSlidePosition = 0;
    private double outtakeClawPosition = 0;
    private double outtakeArmPivotPosition = 1;

    // Transfer Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private double transferCompletionTime = -1;

    private Robot robot;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializePositions();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            mainControls();
            drivetrainControls();

            switch(currentMode) {
                case DRIVE:
                    driveControls();
                case INTAKE:
                    intakeControls();
                    break;
                case OUTTAKE:
                    outtakeControls();
                    break;
            }

            handleDrivetrain();
            handleTransfer();
            handleIntake();
            handleOuttake();

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
        robot.intakeClaw.grab();
        robot.clawPivot.flipTo(0.5);
    }

    private boolean aDown = false, bDown = false, xDown = false, yDown = false;
    private void mainControls() {
        if(gamepad1.a && !aDown) {
            if(currentMode == Mode.INTAKE) {
                intakeSlidePosition = 0;
                intakeClawPosition = 0.575;
                intakeArmPivotPosition = 0;
                intakeClawPivotPosition = 0.5;
                currentMode = Mode.DRIVE;
            }
            else {
                intakeSlidePosition = 1;
                intakeClawPosition = 0.3;
                intakeArmPivotPosition = 1;
                intakeClawPivotPosition = 0.5;
                currentMode = Mode.INTAKE;
            }
        }
        if(gamepad1.b && !bDown) {
            if(currentMode == Mode.OUTTAKE) {
                outtakeSlidePosition = 0;
                outtakeArmPivotPosition = 1;
                outtakeClawPosition = 0.3;
                currentMode = Mode.DRIVE;
            }
            else {
                outtakeSlidePosition = 3000;
                outtakeArmPivotPosition = 1;
                currentMode = Mode.OUTTAKE;
            }
        }
        if(gamepad1.x && !xDown) {
            outtakeClawPosition = 0.85;
            transferCompletionTime = runtime.seconds() + 0.3;
        }
        if(gamepad1.y && !yDown) {
            drive.setPoseEstimate(new Pose2d());
        }
        aDown = gamepad1.a;
        bDown = gamepad1.b;
        xDown = gamepad1.x;
        yDown = gamepad1.y;
    }

    private void drivetrainControls() {
        if(gamepad2.a) {
            initialize();
        }

        if(gamepad1.left_stick_button || gamepad1.right_stick_button) {
            speedReduction = ((double)1/3);
        }
        else {
            speedReduction = 1;
        }
    }

    private boolean leftBDown = false, upDDown = false, downDDown = false, leftDDown = false, rightDDown = false;

    private void driveControls() {
        if(gamepad1.dpad_up && !upDDown) {
            if(intakeClawPosition==0.575) {
                intakeClawPosition=0.3;
            }
            else {
                intakeClawPosition=0.575;
            }
        }
        if(gamepad1.dpad_down && !downDDown) {
            if(intakeArmPivotPosition == 0) {
                intakeClawPosition = 0.3;
                intakeArmPivotPosition = 1;
            }
            else {
                intakeArmPivotPosition = 0;
            }
        }
        upDDown = gamepad1.dpad_up;
        downDDown = gamepad1.dpad_down;
    }

    private void intakeControls() {
        if(gamepad1.left_trigger > 0.1) {
            intakeSlidePosition=Math.min(intakeSlidePosition+gamepad1.left_trigger/10+0.1, 1);
        }
        if(gamepad1.right_trigger > 0.1) {
            intakeSlidePosition=Math.max(intakeSlidePosition-gamepad1.right_trigger/10, 0);
        }
        if(gamepad1.left_bumper && !leftBDown) {
            if(intakeArmPivotPosition == 0.85) {
                intakeArmPivotPosition = 1;
            }
            else {
                intakeArmPivotPosition=0.85;
            }
        }
        if(gamepad1.dpad_up && !upDDown) {
            if(intakeClawPosition==0.575) {
                intakeClawPosition=0.3;
            }
            else {
                intakeClawPosition=0.575;
            }
        }
        if(gamepad1.dpad_down) {
            intakeClawPivotPosition=0.5;
        }
        if(gamepad1.dpad_left && !leftDDown) {
            intakeClawPivotPosition=Math.min(intakeClawPivotPosition+0.1, 1);
        }
        if(gamepad1.dpad_right && !rightDDown) {
            intakeClawPivotPosition=Math.max(intakeClawPivotPosition-0.1, 0);
        }
        leftBDown = gamepad1.left_bumper;
        upDDown = gamepad1.dpad_up;
        leftDDown = gamepad1.dpad_left;
        rightDDown = gamepad1.dpad_right;
    }

    private void outtakeControls() {
        if(gamepad1.left_bumper) {
            outtakeArmPivotPosition = 0.6;
        }
        if(gamepad1.left_trigger>0.2) {
            outtakeArmPivotPosition = 1;
        }
        if(gamepad1.dpad_up && !upDDown) {
            if(outtakeClawPosition == 0.85) {
                outtakeClawPosition = 0.6;
            }
            else {
                outtakeClawPosition = 0.85;
            }
        }
        upDDown = gamepad1.dpad_up;
    }

    private void handleDrivetrain() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y*speedReduction,
                -gamepad1.left_stick_x*speedReduction
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x*speedReduction
                )
        );
    }

    private void handleTransfer() {
        if(transferCompletionTime!=-1 && runtime.seconds() >= transferCompletionTime) {
            transferCompletionTime = -1;
            intakeClawPosition = 0.3;
        }
    }

    private void handleIntake() {
        robot.horizontalSlide.goTo(intakeSlidePosition);
        robot.intakeClaw.openTo(intakeClawPosition);
        robot.intakePivot.flipTo(intakeArmPivotPosition);
        robot.clawPivot.flipTo(intakeClawPivotPosition);
    }

    private void handleOuttake() {
        robot.verticalSlide.goTo(outtakeSlidePosition, 1);
        robot.outtakeClaw.openTo(outtakeClawPosition);
        robot.outtakePivot.flipTo(outtakeArmPivotPosition);
    }

    private void updateTelemetry() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
