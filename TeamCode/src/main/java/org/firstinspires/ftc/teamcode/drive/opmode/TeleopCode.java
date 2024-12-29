package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;


@TeleOp()
public class TeleopCode extends LinearOpMode {

    // Declare OpMode members for the drivetrain
    double xyP = 0.35;
    double headingP = 0.5;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime turnRuntime = new ElapsedTime();
    public ElapsedTime fullMatchRuntime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo servoClaw;
    private Servo servoLeft = null;
    private Servo servoRight = null;
    private Servo servoPivot = null;
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClawLeft;
    private Servo outtakeClawRight;

    //Horizontal Servos
    private Servo leftHorizontal;
    private Servo rightHorizontal;


    // Declare OpMode members for the linear slides
    private DcMotor leftVertical = null;
    private DcMotor rightVertical = null;

    public int linearSlideRetraction = 50;
    public int linearSlideExtension = 1000;

    @Override
    public void runOpMode() throws InterruptedException{


        // Initialize the hardware variables for the drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d autoPose = PoseStorage.currentPose;
        drive.setPoseEstimate(new Pose2d(autoPose.getX(), autoPose.getY(), autoPose.getHeading() - (Math.PI/2)));
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoLeft.scaleRange(0.06, 0.8);
        servoRight.scaleRange(0.06, 0.8);
        servoLeft.setPosition(0);
        servoRight.setPosition(0);
        double posFieldEdge = 0.75;
        double posUnderBar = 0.85;

        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        servoPivot.setDirection(Servo.Direction.FORWARD);
        servoPivot.scaleRange(0, 1);
        servoPivot.setPosition(0.46);

        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        servoClaw.setDirection(Servo.Direction.REVERSE);
        servoClaw.scaleRange(0, 0.4); //replace this later on
        servoClaw.setPosition(0);

        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
        outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
        outtakeClawRight.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight.setDirection(Servo.Direction.REVERSE);
        outtakeClawLeft.scaleRange(0.1, 0.45);
        outtakeClawRight.scaleRange(0.1, 0.45);
        outtakeClawRight.setPosition(0);
        outtakeClawLeft.setPosition(0);
        outtakePivotLeft.setPosition(0.975);
        outtakePivotRight.setPosition(1);

        // Initialize the motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the hardware variables for the linear slides
        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set up horizontal slides
        leftHorizontal = hardwareMap.get(Servo.class, "leftHorizontal");
        rightHorizontal = hardwareMap.get(Servo.class, "rightHorizontal");
        leftHorizontal.setDirection(Servo.Direction.FORWARD);
        rightHorizontal.setDirection(Servo.Direction.REVERSE);
        leftHorizontal.scaleRange(0, 0.5);
        rightHorizontal.scaleRange(0, 0.5);
        double horizontalSlidePosition = 0;
        leftHorizontal.setPosition(horizontalSlidePosition);
        rightHorizontal.setPosition(horizontalSlidePosition);


        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // run until the end of the match (driver presses STOP)
        boolean isFullHorizontalExtent = false;
        boolean isFullHorizontalRetract = false;
        boolean isFullVerticalRetract = false;
        boolean isFullVerticalExtent = false;
        boolean isLocked = false;
        boolean drivingMode = false;
        //false is field centric, true is robot centric
        boolean pressedLastIteration = false;
        boolean pressedSpeedLastIteration = false;
        boolean pressedPivotServoLastIteration = false;
        double pivotServoPosition = 0.46;
        boolean pressedPickServoLastIteration = false;
        boolean pickServoSwitch = false;
        double speed = 1.0;
        boolean isLockTime = false;
        boolean pressedOuttakeClawLastIteration = false;
        boolean pressedOuttakePivotLastIteration = false;
        boolean pressedSpecimenIntakeLastIteration = false;
        boolean outtakeClawPosition = false;
        boolean outtakePivotPosition = false;
        double timeForTransfer = 0;
        boolean waitForClose = false;
        Pose2d setPos = drive.getPoseEstimate();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        turnRuntime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.update();
            double botHeading = drive.getPoseEstimate().getHeading();
            if (botHeading > Math.PI) {
                botHeading = botHeading - (2 * Math.PI);
            }

            telemetry.addData("Horizontal Slide Position", horizontalSlidePosition);
            telemetry.addData("Left Servo Position", servoLeft.getPosition());
            telemetry.addData("Right Servo Position", servoRight.getPosition());
            telemetry.addData("leftVertical", -leftVertical.getCurrentPosition());
            telemetry.addData("rightVertical", -rightVertical.getCurrentPosition());
            telemetry.addData("isLocked", isLocked);
            telemetry.addData("Speed", speed);
            telemetry.addData("Driving Mode (false = fieldcentric, true=robotcentric)", drivingMode);
            telemetry.addData("Heading (degrees)", Math.toDegrees(botHeading));
            telemetry.addData("Bot Position (poseEstimate)", drive.getPoseEstimate().toString());
            telemetry.update();


            // Drive train control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad2.a && !isLocked) {
                Pose2d tempPose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(tempPose.getX(), tempPose.getY(), 0));
            }

            double rotX = (x * Math.cos(-botHeading)) - (y * Math.sin(-botHeading));
            double rotY = (x * Math.sin(-botHeading)) + (y * Math.cos(-botHeading));
            rotX = rotX * 1.1;
            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontPower = (rotY + rotX + rx) / denom;
            double leftBackPower = (rotY - rotX + rx) / denom;
            double rightFrontPower = (rotY - rotX - rx) / denom;
            double rightBackPower = (rotY + rotX - rx) / denom;

            if (isLocked && leftFrontPower == 0.0 && rightFrontPower == 0.0 && leftBackPower == 0.0 && rightBackPower == 0.0) {
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                isLockTime = true;
            } else {
                leftFrontDrive.setPower(leftFrontPower * speed);
                rightFrontDrive.setPower(rightFrontPower * speed);
                leftBackDrive.setPower(leftBackPower * speed);
                rightBackDrive.setPower(rightBackPower * speed);
                setPos = drive.getPoseEstimate();
                runtime.reset();
                isLockTime = false;
            }

            if (runtime.seconds() > 0.8 && runtime.seconds() < 1) {
                setPos = drive.getPoseEstimate();
            }else if (isLockTime && runtime.seconds() > 1){
                lockTo(drive, setPos);
                drive.update();
            }

            //Linear slide control
            boolean motorHorizontalButton = (gamepad1.left_trigger > 0.2);
            boolean motorHorizontalRetract = gamepad1.left_bumper;
            boolean motorHorizontalFullExtension = gamepad1.dpad_up;
            boolean motorHorizontalFullRetraction = gamepad2.touchpad || gamepad1.dpad_down;
            float motorVerticalButton = gamepad1.right_trigger;
            boolean motorVerticalRetract = gamepad1.right_bumper;
            boolean motorVerticalFullExtension = gamepad1.y;
            boolean motorVerticalFullRetraction = (gamepad2.left_trigger > 0.2)||gamepad1.a;
            boolean lockDriveTrain = gamepad1.left_stick_button;
            boolean speedButton = gamepad2.left_stick_button;
            boolean servoOut = (gamepad2.right_trigger > 0.2);
            boolean servoIn = gamepad1.dpad_right;
            boolean servoPivotRight = gamepad2.dpad_right;
            boolean servoPivotLeft = gamepad2.dpad_left;
            boolean servoPivotReset = gamepad2.dpad_down;
            boolean servoPickButton = gamepad2.dpad_up;
            boolean fieldEdgeButton = gamepad2.left_bumper;
            //boolean underBarButton = gamepad2.right_bumper;
            boolean prepareIntakeButton = gamepad2.right_bumper;
            boolean outtakeClawButton = gamepad2.y;
            boolean outtakePivotButton = gamepad2.x;
            boolean highBucketButton = gamepad2.b;
            boolean specimenGrabButton = gamepad2.back;

            if (horizontalSlidePosition < 0.95 && motorHorizontalButton) {
                horizontalSlidePosition += 0.1;
            } else if (horizontalSlidePosition > 0.05 && motorHorizontalRetract) {
                horizontalSlidePosition -= 0.1;
            } else if (motorHorizontalFullExtension) {
                horizontalSlidePosition = 1;
            } else if (motorHorizontalFullRetraction) {
                horizontalSlidePosition = 0;
            }
            leftHorizontal.setPosition(horizontalSlidePosition);
            rightHorizontal.setPosition(horizontalSlidePosition);

            if(specimenGrabButton && !pressedSpecimenIntakeLastIteration){
                if (!outtakePivotPosition){
                    outtakePivotLeft.setPosition(0.215);
                    outtakePivotRight.setPosition(0.225);
                    outtakePivotPosition = true;
                } else {
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1.05);
                    outtakePivotPosition = false;
                }
            }
            pressedSpecimenIntakeLastIteration = specimenGrabButton;


            if (highBucketButton) {
                outtakePivotLeft.setPosition(0.625);
                outtakePivotRight.setPosition(0.65);
                outtakePivotPosition = true;
            }

            if ((outtakePivotButton) && !pressedOuttakePivotLastIteration){
                if (!outtakePivotPosition){
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                    outtakePivotPosition = true;
                } else {
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1.05);
                    outtakePivotPosition = false;
                }
            }
            pressedOuttakePivotLastIteration = outtakePivotButton;

            if (outtakeClawButton && !pressedOuttakeClawLastIteration){
                if (!outtakeClawPosition){
                    outtakeClawLeft.setPosition(1);
                    outtakeClawRight.setPosition(1);
                    waitForClose = true;
                    timeForTransfer = fullMatchRuntime.seconds() + 0.3;
                    outtakeClawPosition = true;
                } else {
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);
                    outtakeClawPosition = false;
                }

            }
            pressedOuttakeClawLastIteration = outtakeClawButton;

            if (servoPickButton && !pressedPickServoLastIteration){
                if (pickServoSwitch){
                    servoClaw.setPosition(0); //replace this later
                    pickServoSwitch = false;
                } else{
                    servoClaw.setPosition(1); // replace this later
                    pickServoSwitch = true;
                }
            }

            if (waitForClose && fullMatchRuntime.seconds() >= timeForTransfer){
                waitForClose = false;
                servoClaw.setPosition(1); //replace this later
                pickServoSwitch = true;
            }
            pressedPickServoLastIteration = servoPickButton;

            if (servoIn){
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
            }else if(prepareIntakeButton){
                servoLeft.setPosition(0.7);
                servoRight.setPosition(0.7);
            }
            else if(servoOut){
                servoLeft.setPosition(1);
                servoRight.setPosition(1);
                servoClaw.setPosition(1); //replace this later
                pickServoSwitch = true;
            } else if (fieldEdgeButton) {
                servoLeft.setPosition(posFieldEdge);
                servoRight.setPosition(posFieldEdge);
            } /*else if (underBarButton){
                servoLeft.setPosition(posUnderBar);
                servoRight.setPosition(posUnderBar);
            }*/

            if (servoPivotLeft && !pressedPivotServoLastIteration && pivotServoPosition < 1){
                pivotServoPosition += 0.1;
                servoPivot.setPosition(pivotServoPosition);
            } else if (servoPivotRight && !pressedPivotServoLastIteration && pivotServoPosition > 0){
                pivotServoPosition -= 0.1;
                servoPivot.setPosition(pivotServoPosition);
            } else if (servoPivotReset){
                pivotServoPosition = 0.5;
                servoPivot.setPosition(pivotServoPosition);
            }

            pressedPivotServoLastIteration = servoPivotLeft || servoPivotRight;

            if (lockDriveTrain && !pressedLastIteration){
                isLocked = !isLocked;
                if (isLocked){
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    drive.update();
                    setPos = drive.getPoseEstimate();
                    runtime.reset();
                    isLockTime = false;
                }
            }


            if (speedButton && !pressedSpeedLastIteration){
                if (speed > 0.9){
                    speed /= 3;
                }
                else{
                    speed = 1.0;
                }
            }
            pressedSpeedLastIteration = speedButton;

            float leftVerticalPosition = -leftVertical.getCurrentPosition();
            float rightVerticalPosition = -rightVertical.getCurrentPosition();

            if ((motorVerticalButton != 0) && (leftVerticalPosition <= 3050) && (rightVerticalPosition >= -3050)) {
                leftVertical.setPower(speed * motorVerticalButton);
                rightVertical.setPower(speed * -motorVerticalButton);
            } else if ((motorVerticalRetract) && ((leftVerticalPosition >= 50) || leftVerticalPosition <= -50) && (rightVerticalPosition >= 50 || rightVerticalPosition <= -50)) {
                leftVertical.setPower(-speed);
                rightVertical.setPower(speed);
            } else if (motorVerticalFullExtension){
                isFullVerticalExtent = true;
                leftVertical.setPower(1.5);
                rightVertical.setPower(-1.5);
            } else if (motorVerticalFullRetraction) {
                isFullVerticalRetract = true;
                leftVertical.setPower(-1.5);
                rightVertical.setPower(1.5);
            } else if (!(isFullVerticalExtent || isFullVerticalRetract)) {
                leftVertical.setPower(0);
                rightVertical.setPower(0);
            } else if (isFullVerticalExtent && !(leftVerticalPosition <= 3050 && rightVerticalPosition >= -3050)){
                isFullVerticalExtent = false;
            } else if (isFullVerticalRetract && !(((leftVerticalPosition >= 50) || leftVerticalPosition <= -50) && (rightVerticalPosition >= 50 || rightVerticalPosition <= -50))) {
                isFullVerticalRetract = false;
                leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        }
    }
    public void lockTo(SampleMecanumDrive drive, Pose2d targetPos){
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading() - Angle.normDelta(currPos.getHeading()));
        drive.setWeightedDrivePower(new Pose2d(xy.getX() * 0.2, xy.getY() * xyP, heading * headingP));
    }
    public int angleChecker(double start, double end, double cur){
        if (start < end){
            if (start <= cur && cur <= end){
                return 1;
            } else {
                return -1;
            }
        } else {
            if (end <= cur && cur <= start){
                return -1;
            } else {
                return 1;
            }
        }
    }
}