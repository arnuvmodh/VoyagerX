package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp()
public class testCodeVertSlides extends LinearOpMode {

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
    private IMU fieldcentricimu = null;
    private Servo servoLeft = null;
    private Servo servoRight = null;
    private Servo servoPivot = null;
    private Servo servoClawLeft = null;
    private Servo servoClawRight = null;
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClawLeft;
    private Servo outtakeClawRight;

    // Declare OpMode members for the linear slides
    private DcMotor leftHorizontal = null;
    private DcMotor rightHorizontal = null;
    private DcMotor leftVertical = null;
    private DcMotor rightVertical = null;

    public int linearSlideRetraction = 50;
    public int linearSlideExtension = 1000;

    @Override
    public void runOpMode() throws InterruptedException{


        fieldcentricimu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        fieldcentricimu.initialize(parameters);


        // Initialize the hardware variables for the drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoLeft.scaleRange(0, 0.65);
        servoRight.scaleRange(0, 0.65);
        servoLeft.setPosition(0);
        servoRight.setPosition(0);
        double posFieldEdge = 0.75;
        double posUnderBar = 0.9;

        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        servoPivot.setDirection(Servo.Direction.FORWARD);
        servoPivot.scaleRange(0, 1);
        servoPivot.setPosition(0.5);

        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");
        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.REVERSE);
        servoClawLeft.setPosition(0.575);
        servoClawRight.setPosition(0.575);

        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
        outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
        outtakeClawRight.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight.setDirection(Servo.Direction.REVERSE);
        outtakeClawLeft.scaleRange(0, 0.55);
        outtakeClawRight.scaleRange(0, 0.55);
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
        leftHorizontal = hardwareMap.get(DcMotor.class, "left_slide");
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontal = hardwareMap.get(DcMotor.class, "right_slide");
        rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set up encoders for horizontal slides
        leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
        double pivotServoPosition = 0.5;
        boolean pressedPickServoLastIteration = false;
        boolean pickServoSwitch = false;
        double speed = 1.0;
        boolean isLockTime = false;
        boolean pressedOuttakeClawLastIteration = false;
        boolean pressedOuttakePivotLastIteration = false;
        boolean outtakeClawPosition = false;
        boolean outtakePivotPosition = false;
        double timeForTransfer = 0;
        boolean waitForClose = false;
        Pose2d setPos = drive.getPoseEstimate();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        fieldcentricimu.resetYaw();
        runtime.reset();
        turnRuntime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double botHeading = fieldcentricimu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("leftHorizontal", leftHorizontal.getCurrentPosition());
            telemetry.addData("rightHorizontal", rightHorizontal.getCurrentPosition());
            telemetry.addData("Left Servo Position", servoLeft.getPosition());
            telemetry.addData("Right Servo Position", servoRight.getPosition());
            telemetry.addData("leftVertical", -leftVertical.getCurrentPosition());
            telemetry.addData("rightVertical", -rightVertical.getCurrentPosition());
            telemetry.addData("isLocked", isLocked);
            telemetry.addData("Speed", speed);
            telemetry.addData("Driving Mode (false = fieldcentric, true=robotcentric)", drivingMode);
            telemetry.addData("IMU Value", Math.toDegrees(botHeading));
            telemetry.update();


            // Drive train control
            drive.update();
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad2.a) {
                fieldcentricimu.resetYaw();
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

            // Linear slide control
            float motorHorizontalButton = gamepad1.left_trigger;
            boolean motorHorizontalRetract = gamepad1.left_bumper;
            boolean motorHorizontalFullExtension = gamepad1.dpad_up;
            boolean motorHorizontalFullRetraction = gamepad1.dpad_down;
            float motorVerticalButton = gamepad1.right_trigger;
            boolean motorVerticalRetract = gamepad1.right_bumper;
            boolean motorVerticalFullExtension = gamepad1.y;
            boolean motorVerticalFullRetraction = gamepad1.a;
            boolean lockDriveTrain = gamepad1.left_stick_button;
            boolean speedButton = gamepad1.back;
            boolean servoOut = gamepad1.dpad_left;
            boolean servoIn = gamepad1.dpad_right;
            boolean servoPivotRight = gamepad2.dpad_right;
            boolean servoPivotLeft = gamepad2.dpad_left;
            boolean servoPivotReset = gamepad2.dpad_down;
            boolean servoPickButton = gamepad2.dpad_up;
            boolean fieldEdgeButton = gamepad2.left_bumper;
            boolean underBarButton = gamepad2.right_bumper;
            boolean outtakeClawButton = gamepad2.y;
            boolean outtakePivotButton = gamepad2.x;
            boolean highBucketButton = gamepad2.b;

            if (highBucketButton) {
                outtakePivotLeft.setPosition(0.625);
                outtakePivotRight.setPosition(0.65);
                outtakePivotPosition = true;
            }

            if (outtakePivotButton && !pressedOuttakePivotLastIteration){
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
                    servoClawLeft.setPosition(0.575);
                    servoClawRight.setPosition(0.575);
                    pickServoSwitch = false;
                } else{
                    servoClawLeft.setPosition(0.3);
                    servoClawRight.setPosition(0.3);
                    pickServoSwitch = true;
                }
            }

            if (waitForClose && fullMatchRuntime.seconds() >= timeForTransfer){
                waitForClose = false;
                servoClawLeft.setPosition(0.3);
                servoClawRight.setPosition(0.3);
                pickServoSwitch = true;
            }
            pressedPickServoLastIteration = servoPickButton;

            if (servoIn){
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
            } else if(servoOut){
                servoLeft.setPosition(1);
                servoRight.setPosition(1);
                servoClawLeft.setPosition(0.3);
                servoClawRight.setPosition(0.3);
                pickServoSwitch = true;
            } else if (fieldEdgeButton) {
                servoLeft.setPosition(posFieldEdge);
                servoRight.setPosition(posFieldEdge);
            } else if (underBarButton){
                servoLeft.setPosition(posUnderBar);
                servoRight.setPosition(posUnderBar);
            }

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

            float leftHorizontalPosition = leftHorizontal.getCurrentPosition();
            float rightHorizontalPosition = rightHorizontal.getCurrentPosition();
            float leftVerticalPosition = -leftVertical.getCurrentPosition();
            float rightVerticalPosition = -rightVertical.getCurrentPosition();
            if ((motorHorizontalButton != 0) && (leftHorizontalPosition <= linearSlideExtension) && (rightHorizontalPosition >= -linearSlideExtension)) {
                leftHorizontal.setPower(speed * motorHorizontalButton);
                rightHorizontal.setPower(speed * -motorHorizontalButton);
            } else if ((motorHorizontalRetract) && ((leftHorizontalPosition >= linearSlideRetraction) || leftHorizontalPosition <= -linearSlideRetraction) && (rightHorizontalPosition >= linearSlideRetraction || rightHorizontalPosition <= -linearSlideRetraction)) {
                leftHorizontal.setPower(-speed);
                rightHorizontal.setPower(speed);
            } else if (motorHorizontalFullExtension){
                isFullHorizontalExtent = true;
                leftHorizontal.setPower(1.5);
                rightHorizontal.setPower(-1.5);
                servoLeft.setPosition(1);
                servoRight.setPosition(1);
                servoClawLeft.setPosition(0.3);
                servoClawRight.setPosition(0.3);
                pickServoSwitch = true;
            } else if (motorHorizontalFullRetraction) {
                isFullHorizontalRetract = true;
                leftHorizontal.setPower(-1.5);
                rightHorizontal.setPower(1.5);
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
                pivotServoPosition = 0.5;
                servoPivot.setPosition(pivotServoPosition);
                servoClawLeft.setPosition(0.575);
                servoClawRight.setPosition(0.575);
                pickServoSwitch = false;
            } else if (!(isFullHorizontalExtent || isFullHorizontalRetract)) {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
            } else if (isFullHorizontalExtent && !(leftHorizontal.getCurrentPosition() <= linearSlideExtension && rightHorizontal.getCurrentPosition() >= -linearSlideExtension)){
                isFullHorizontalExtent = false;
            } else if (isFullHorizontalRetract && !(((leftHorizontalPosition >= linearSlideRetraction) || leftHorizontalPosition <= -linearSlideRetraction) && (rightHorizontalPosition >= linearSlideRetraction || rightHorizontalPosition <= -linearSlideRetraction))){
                isFullHorizontalRetract = false;
                leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

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





