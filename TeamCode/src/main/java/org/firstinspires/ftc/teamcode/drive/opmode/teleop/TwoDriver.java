package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@TeleOp
public class TwoDriver extends LinearOpMode{

    // Intake Control Variables
    private double intakeSlidePosition = 0;
    private double spintakePower = 0;
    private double intakeArmPivotPosition = 0.25;
    private double intakeClawPivotPosition = 0.5;

    // Outtake Control Variables
    private int outtakeSlidePosition = 0;
    private double outtakeClawPosition = 0;
    private double outtakeArmPivotPosition = 1;

    // Transfer Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private double transferCompletionTime = -1;
    private double outtakeCompletionTime = -1;
    private double outtakeArmCompletionTime = -1;

    private Robot robot;
    private SampleMecanumDrive drive;
    private boolean locked = false;
    private Pose2d setPos;
    private double speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializePositions();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            drivetrainControls();
            intakeControls();
            outtakeControls();

            handleDrivetrain();
            handleTransfer();
            handleIntake();
            handleOuttake();

            drive.update();
            updateTelemetry();
        }
    }

    private boolean aDown = false, bDown = false, xDown = false, yDown = false;
    private boolean leftBDown = false, rightBDown = false;
    private boolean upDDown = false, downDDown = false, leftDDown = false, rightDDown = false;


    private void updateButtons() {
        aDown = gamepad1.a;
        bDown = gamepad1.b;
        xDown = gamepad1.x;
        yDown = gamepad1.y;

        leftBDown = gamepad1.left_bumper;
        rightBDown = gamepad1.right_bumper;

        upDDown = gamepad1.dpad_up;
        downDDown = gamepad1.dpad_down;
        leftDDown = gamepad1.dpad_left;
        rightDDown = gamepad1.dpad_right;
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
        robot.intakePivot.flipTo(0);
        robot.outtakeClaw.release();
        robot.clawPivot.flipTo(0.5);
        robot.hangPivot.setPosition(0.5);
    }

    private void drivetrainControls() {
        if(gamepad2.a) {
            speed = ((double)1/3);
        }
        else {
            speed = 1;
        }
        if(gamepad1.left_stick_button) {
            intakeSlidePosition = 1;
            intakeArmPivotPosition = 0.655;
            spintakePower = 0;
        }
        if(gamepad1.right_stick_button) {
            outtakeSlidePosition = 0;
            intakeSlidePosition = 0;
            intakeArmPivotPosition = 0.25;
            spintakePower = 0;
            intakeClawPivotPosition = 0.5;
            outtakeArmPivotPosition = 1;
            outtakeClawPosition = 0.3;
        }
    }

    private boolean shareDown = false;
    private void intakeControls() {
        if(gamepad1.left_trigger>0.2) {
            intakeArmPivotPosition = 0.7325;
            spintakePower = 1;
        }

        if(gamepad1.share || gamepad2.share) {
            spintakePower = -1;
        }
        else if(shareDown && spintakePower!=1) {
            spintakePower = 0;
        }
        shareDown = gamepad1.share || gamepad2.share;

        if(gamepad1.left_bumper && !leftBDown) {
            if(outtakeArmPivotPosition != 0.6) {
                outtakeArmPivotPosition = 0.6;
                outtakeArmCompletionTime = runtime.seconds()+0.3;
            }
            else {
                outtakeArmPivotPosition = 1;
            }
        }
        leftBDown = gamepad1.left_bumper;

        if(gamepad2.dpad_up) {
            drive.setPoseEstimate(new Pose2d());
        }
        if(gamepad1.dpad_left && !leftDDown) {
            intakeClawPivotPosition=Math.min(intakeClawPivotPosition+0.1, 1);
        }
        leftDDown = gamepad1.dpad_left;

        if(gamepad1.dpad_right && !rightDDown) {
            intakeClawPivotPosition=Math.max(intakeClawPivotPosition-0.1, 0);
        }
        rightDDown = gamepad1.dpad_right;

        if(gamepad1.dpad_down) {
            intakeClawPivotPosition=0.5;
        }
    }

    private boolean touchpadDown = false;
    private boolean rightTriggerDown = false;
    private void outtakeControls() {
        if(gamepad1.touchpad && !touchpadDown) {
            if(locked) {
                locked = false;
            }
            else {
                setPos = drive.getPoseEstimate();
                locked = true;
            }
        }
        touchpadDown = gamepad1.touchpad;


        if(gamepad1.x && !xDown) {
            if(spintakePower == 0) {
                spintakePower = 1;
            }
            else {
                spintakePower = 0;
            }
        }
        xDown = gamepad1.x;

        if(gamepad1.y && !yDown) {
            outtakeSlidePosition = 1975;
        }
        yDown = gamepad1.y;

        if(gamepad1.a && !aDown) {
            if(outtakeArmPivotPosition != 0.2){
                outtakeArmPivotPosition = 0.2;
            }
            else {
                outtakeArmPivotPosition = 1;
            }
        }
        aDown = gamepad1.a;

        if(gamepad1.b) {
            robot.verticalSlide.extend(1);
            outtakeSlidePosition = -1;
        }
        else if (!gamepad1.b && bDown){
            robot.verticalSlide.reset();
        }
        bDown = gamepad1.b;

        if((gamepad1.right_trigger > 0.2 || gamepad2.right_trigger>0.2) && !rightTriggerDown) {
            if(robot.hangPivot.getPosition()==0.5) {
                robot.hangPivot.setPosition(0);
            }
            else {
                robot.hangPivot.setPosition(0.5);
            }
        }
        rightTriggerDown = (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger>0.2);

        if(gamepad1.right_bumper && !rightBDown) {
            if(outtakeClawPosition == 0.85) {
                outtakeClawPosition = 0.3;
            }
            else {
                outtakeClawPosition = 0.85;
                transferCompletionTime = runtime.seconds() + 0.3;
            }
        }
        rightBDown = gamepad1.right_bumper;

    }

    public void lockTo(SampleMecanumDrive drive, Pose2d targetPos){
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading() - Angle.normDelta(currPos.getHeading()));
        drive.setWeightedDrivePower(new Pose2d(xy.getX() * 0.2, xy.getY() * 0.35, heading * 0.5));
    }

    private void handleDrivetrain() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        if(locked) {
            lockTo(drive, setPos);
        }
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX()*speed,
                            input.getY()*speed,
                            -gamepad1.right_stick_x*speed
                    )
            );
        }
    }

    private void handleTransfer() {
        if(transferCompletionTime!=-1 && runtime.seconds() >= transferCompletionTime) {
            transferCompletionTime = -1;
            intakeSlidePosition=0.3;
            spintakePower = -1;
            outtakeCompletionTime = runtime.seconds() + 0.1;
        }
        if(outtakeCompletionTime !=-1 && runtime.seconds() >= outtakeCompletionTime) {
            outtakeSlidePosition = 3050;
            outtakeCompletionTime = -1;
        }
    }

    private void handleIntake() {
        robot.horizontalSlide.goTo(intakeSlidePosition);
        robot.spintake.spinIn(spintakePower);
        robot.intakePivot.flipTo(intakeArmPivotPosition);
        robot.clawPivot.flipTo(intakeClawPivotPosition);
    }

    private void handleOuttake() {
        if(outtakeArmCompletionTime !=-1 && runtime.seconds() >= outtakeArmCompletionTime) {
            outtakeClawPosition = 0.3;
            outtakeArmCompletionTime = -1;
        }
        if(outtakeSlidePosition != -1) {
            robot.verticalSlide.goTo(outtakeSlidePosition, 1);
        }
        robot.outtakeClaw.openTo(outtakeClawPosition);
        robot.outtakePivot.flipTo(outtakeArmPivotPosition);
    }

    private void updateTelemetry() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Dpad Left", leftDDown);
        telemetry.addData("Dpad Right", rightDDown);
        telemetry.update();
    }
}
