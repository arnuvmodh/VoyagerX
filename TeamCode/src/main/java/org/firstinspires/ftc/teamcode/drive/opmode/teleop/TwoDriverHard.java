package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@TeleOp
public class TwoDriverHard extends LinearOpMode{

    // Drive Control Variables
    private double speedReduction = 1;
    private int fieldCentricResets = 0;

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
            driveControls();
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
        robot.hangPivot.setPosition(0);
    }

    private void driveControls() {
        if(gamepad2.a && gamepad2.share) {
            fieldCentricResets++;
            drive.setPoseEstimate(new Pose2d());
        }
    }

    private boolean intakeClawPivotPressed = false, intakeClawTogglePressed = false;
    private void intakeControls() {
        boolean intakeSlideOut = gamepad1.dpad_up;
        boolean intakeSlideIn = gamepad1.dpad_down;
        boolean intakeArmPivotOut = (gamepad2.right_trigger > 0.2)||(gamepad1.dpad_left);
        boolean intakeArmPivotIn = (gamepad1.dpad_right || gamepad2.touchpad);
        boolean intakeArmPivotFieldEdge = gamepad2.left_bumper;
        boolean intakeArmPivotUnderBar = gamepad2.right_bumper;
        boolean intakeClawPivotLeft = gamepad2.dpad_left;
        boolean intakeClawPivotRight = gamepad2.dpad_right;
        boolean intakeCLawPivotReset = gamepad2.dpad_down;
        boolean intakeClawToggle = gamepad2.dpad_up;

        if(intakeSlideOut) {
            intakeSlidePosition = 1;
        }
        if(intakeSlideIn) {
            intakeSlidePosition = 0;
        }

        if(intakeArmPivotOut) {
            intakeArmPivotPosition = 1;
        }
        if(intakeArmPivotIn) {
            intakeSlidePosition = 0;
            intakeClawPivotPosition = 0.5;
            intakeArmPivotPosition = 0;
        }
        if(intakeArmPivotFieldEdge) {
            intakeArmPivotPosition = 0.75;
        }
        if(intakeArmPivotUnderBar) {
            intakeArmPivotPosition = 0.85;
        }

        if(intakeClawPivotLeft && !intakeClawPivotPressed) {
            intakeClawPivotPosition = Math.min(intakeClawPivotPosition+0.1, 1);
        }
        if(intakeClawPivotRight && !intakeClawPivotPressed) {
            intakeClawPivotPosition = Math.max(intakeClawPivotPosition-0.1, 0);
        }
        if(intakeCLawPivotReset) {
            intakeClawPivotPosition = 0.5;
        }
        intakeClawPivotPressed = intakeClawPivotLeft || intakeClawPivotRight;

        if(intakeClawToggle && !intakeClawTogglePressed) {
            if(intakeClawPosition != 0.575) {
                intakeClawPosition = 0.575;
            }
            else {
                intakeClawPosition = 0.3;
            }
        }
        intakeClawTogglePressed = intakeClawToggle;

    }

    private boolean transferPressed = false, outtakePivotTogglePressed = false, specimenGrabButtonPressed = false;
    private boolean outtakeSlideExtendDown = false, outtakeSlideRetractDown = false;
    private boolean hangButtonDown = false;
    private void outtakeControls() {
        boolean outtakeSlideExtend = gamepad1.right_trigger > 0.01;
        boolean outtakeSlideRetract = gamepad1.right_bumper;
        boolean outtakeSlideFullExtension = gamepad1.y;
        boolean outtakeSlideFullRetraction = (gamepad2.left_trigger > 0.2)||gamepad1.a;
        boolean outtakeSlideSpecimen = (gamepad1.left_trigger > 0.2);

        boolean transfer = gamepad2.y;
        boolean outtakePivotToggle = gamepad2.x;
        boolean highBucketButton = gamepad2.b;
        boolean specimenGrabButton = gamepad2.back;

        boolean hangButton = gamepad2.right_stick_button;

        if(hangButton && !hangButtonDown) {
            if(robot.hangPivot.getPosition() != 0.5) {
                robot.hangPivot.setPosition(0.5);
            }
            else {
                robot.hangPivot.setPosition(0);
            }
        }
        hangButtonDown = hangButton;

        if(outtakeSlideExtend) {
            robot.verticalSlide.extend(1);
            outtakeSlidePosition = -1;
        }
        else if(outtakeSlideExtendDown) {
            robot.verticalSlide.reset();
        }
        outtakeSlideExtendDown = outtakeSlideExtend;

        if(outtakeSlideRetract) {
            robot.verticalSlide.extend(1);
            outtakeSlidePosition = -1;
        }
        else if(outtakeSlideRetractDown) {
            robot.verticalSlide.reset();
        }
        outtakeSlideRetractDown = outtakeSlideRetract;

        if(outtakeSlideFullExtension) {
            outtakeSlidePosition = 3050;
        }
        if(outtakeSlideFullRetraction) {
            outtakeSlidePosition = 0;
        }
        if(outtakeSlideSpecimen) {
            outtakeSlidePosition = 1950;
        }

        if(transfer && !transferPressed) {
            if(outtakeClawPosition < 0.85) {
                outtakeClawPosition = 0.85;
                transferCompletionTime = runtime.seconds() + 0.3;
            }
            else {
                outtakeClawPosition = 0;
            }
        }
        transferPressed = transfer;

        if(outtakePivotToggle && !outtakePivotTogglePressed) {
            if(outtakeArmPivotPosition != 1) {
                outtakeArmPivotPosition = 1;
            } else {
                outtakeArmPivotPosition = 0.5;
            }
        }
        outtakePivotTogglePressed = outtakePivotToggle;

        if(highBucketButton) {
            outtakeArmPivotPosition = 0.65;
        }
        if(specimenGrabButton && !specimenGrabButtonPressed){
            if(outtakeArmPivotPosition!=1){
                outtakeArmPivotPosition = 1;
            } else {
                outtakeArmPivotPosition = 0.2;
            }
        }
        specimenGrabButtonPressed = specimenGrabButton;
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
        telemetry.addData("Field Centric Resets", fieldCentricResets);
        telemetry.update();
    }
}
