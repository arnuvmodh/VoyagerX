package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    private double intakeRetractionTime = -1;
    private double transferCompletionTime = -1;
    private double outtakeSampleCompletionTime = -1;
    private double outtakeSpecimenCompletionTime = -1;
    private double outtakeArmCompletionTime = -1;
    private boolean outtakeSpecimen = false;

    private String oppositeColor = "Blue";

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

    private boolean aDown = false, bDown = false;
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
        robot.intakePivot.flipTo(0.25);
        robot.outtakeClaw.release();
        robot.clawPivot.flipTo(0.5);
        robot.hangPivot.setPosition(0.5);
    }

    private void drivetrainControls() {
        if(gamepad2.right_stick_button) {

        }
        if(gamepad2.a) {
            speed = ((double)1/3);
        }
        else {
            speed = 1;
        }
        if(gamepad1.left_stick_button || gamepad2.right_bumper) {
            intakeSlidePosition = 1;
            intakeArmPivotPosition = 0.655;
            spintakePower = 0;
        }
        if(gamepad1.right_stick_button) {
            outtakeSlidePosition = 0;
            intakeSlidePosition = 0.15;
            intakeArmPivotPosition = 0.25;
            spintakePower = 0;
            intakeClawPivotPosition = 0.5;
            outtakeArmPivotPosition = 1;
            outtakeClawPosition = 0.3;
        }
    }

    private boolean shareDown = false;
    private boolean g2dpadDDown = false;
    private void intakeControls() {
        if(gamepad2.touchpad) {
            intakeArmPivotPosition = 0.25;
        }

        if(gamepad1.left_trigger>0.2 || gamepad2.right_bumper) {
            intakeArmPivotPosition = 0.7325;
            spintakePower = 1;
        }

        if(gamepad2.dpad_down && !g2dpadDDown) {
            if(oppositeColor.equals("Blue")) oppositeColor = "Red";
            else oppositeColor = "Blue";
        }
        g2dpadDDown = gamepad2.dpad_down;

        if(gamepad1.share || (gamepad2.left_trigger>0.2)) {
            spintakePower = -1;
        }
        else if(shareDown && spintakePower!=1) {
            spintakePower = 0;
        }
        shareDown = gamepad1.share || (gamepad2.left_trigger>0.2);

        if(gamepad1.left_bumper && !leftBDown) {
            if(outtakeArmPivotPosition != 0.5) {
                outtakeArmPivotPosition = 0.5;
                outtakeArmCompletionTime = runtime.seconds()+0.15;
            }
            else {
                outtakeArmPivotPosition = 1;
            }
        }
        leftBDown = gamepad1.left_bumper;

        if(gamepad2.right_stick_button) {
            drive.setPoseEstimate(new Pose2d());
        }
        if((gamepad1.dpad_left || gamepad2.dpad_left) && !leftDDown) {
            intakeClawPivotPosition=Math.min(intakeClawPivotPosition+0.1, 1);
        }
        leftDDown = (gamepad1.dpad_left || gamepad2.dpad_left);

        if((gamepad1.dpad_right || gamepad2.dpad_right) && !rightDDown) {
            intakeClawPivotPosition=Math.max(intakeClawPivotPosition-0.1, 0);
        }
        rightDDown = (gamepad1.dpad_right || gamepad2.dpad_right);

        if(gamepad1.dpad_down) {
            intakeClawPivotPosition=0.5;
        }
    }

    private boolean touchpadDown = false;
    private boolean rightTriggerDown = false;
    private void outtakeControls() {
        if(gamepad2.share) {
            outtakeArmPivotPosition = 0.3;
        }
        if(gamepad2.dpad_up) {
            outtakeSlidePosition = 1070;
        }
        if(gamepad2.dpad_down) {
            outtakeSlidePosition = 0;
        }
        if(gamepad2.x) {
            outtakeArmPivotPosition = 0.7;
        }

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

        if((gamepad1.a||gamepad2.y) && !aDown) {
            if(outtakeClawPosition == 0.9) {
                outtakeClawPosition = 0.3;
            }
            else {
                outtakeClawPosition = 0.9;
            }
        }
        aDown = (gamepad1.a||gamepad2.y);

        if(gamepad1.b) {
            outtakeClawPosition = 0.9;
            outtakeSpecimen = true;
            transferCompletionTime = runtime.seconds() + 0.3;
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
            if(outtakeClawPosition == 0.9) {
                outtakeClawPosition = 0.3;
            }
            else {
                outtakeClawPosition = 0.9;
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

    private boolean xDown = false, yDown = false;
    private Pose2d highBasketPose;
    private void handleDrivetrain() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        if(gamepad1.y && !yDown) {
            highBasketPose = drive.getPoseEstimate();
        }
        yDown = gamepad1.y;

        if(gamepad1.x && !xDown) {
            Trajectory highBucketOuttake = drive.trajectoryBuilder(poseEstimate, true)
                    .splineToLinearHeading(highBasketPose, Math.toRadians(-180))
                    .build();
            drive.followTrajectoryAsync(highBucketOuttake);

            outtakeSlidePosition = 0;
            intakeSlidePosition = 0.15;
            intakeArmPivotPosition = 0.25;
            spintakePower = 0;
            intakeClawPivotPosition = 0.5;
            outtakeArmPivotPosition = 1;
            outtakeClawPosition = 0.3;
            intakeRetractionTime = runtime.seconds() + 0.7;
        }
        if(!gamepad1.x) {
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
        xDown = gamepad1.x;
    }

    private void handleTransfer() {
        if(intakeRetractionTime!=-1 && runtime.seconds() >= intakeRetractionTime) {
            outtakeClawPosition = 0.9;
            intakeRetractionTime = -1;
            transferCompletionTime = runtime.seconds() + 0.3;
        }

        if(transferCompletionTime!=-1 && runtime.seconds() >= transferCompletionTime) {
            transferCompletionTime = -1;
            intakeSlidePosition=0.3;
            spintakePower = -1;
            if(outtakeSpecimen) {
                outtakeSpecimenCompletionTime = runtime.seconds() + 0.1;
            }
            else {
                outtakeSampleCompletionTime = runtime.seconds() + 0.1;
            }
        }
        if(outtakeSampleCompletionTime !=-1 && runtime.seconds() >= outtakeSampleCompletionTime) {
            outtakeSlidePosition = 3050;
            outtakeArmPivotPosition = 0.7;
            outtakeSampleCompletionTime = -1;
        }

        if(outtakeSpecimenCompletionTime !=-1 && runtime.seconds() >= outtakeSpecimenCompletionTime) {
            outtakeSlidePosition = 1070;
            outtakeArmPivotPosition = 0.3;
            outtakeSpecimenCompletionTime = -1;
            outtakeSpecimen = false;
        }
    }

    private void handleIntake() {
//        if(spintakePower==1&&robot.colorSensor.getColor().equals(oppositeColor)) {
//            spintakePower=-1;
//        }
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
        if(outtakeSlidePosition == 3050) {
            robot.verticalSlide.extendFull();
        }
        else if(outtakeSlidePosition != -1) {
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
        telemetry.addData("Color", robot.colorSensor.getColorAsString());
        telemetry.addData("Hue", robot.colorSensor.getHue());
        telemetry.update();
    }
}
