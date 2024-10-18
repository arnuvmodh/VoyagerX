package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
public class VoyagersXTeleopCode extends LinearOpMode {

    // Declare OpMode members for the drivetrain
    double xyP = 0.35;
    double headingP = 0.5;
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare OpMode members for the linear slides
    private DcMotor leftHorizontal = null;
    private DcMotor rightHorizontal = null;
    private DcMotor leftVertical = null;
    private DcMotor rightVertical = null;


    public double integralSum = 0;
    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;
    public double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException{


        // Initialize the hardware variables for the drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        leftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)

        // run until the end of the match (driver presses STOP)
        boolean isFullHorizontalExtent = false;
        boolean isFullVerticalExtent = false;
        boolean isFullHorizontalRetract = false;
        boolean isFullVerticalRetract = false;
        boolean isLocked = false;
        boolean drivingMode = false;
        //false is field centric, true is robot centric
        boolean pressedLastIteration = false;
        boolean pressedSpeedLastIteration = false;
        boolean pressed180LastIteration = false;
        boolean pressedStyleSwitchLastIteration = false;
        boolean leftmoving = false;
        boolean rightmoving = false;
        double speed = 1.0;
        double turnToMax = 0;
        double turnToMin = 0;
        double turnTo = 0;
        double leftVerticalStandbyPos = 0;
        double rightVerticalStandbyPos = 0;
        Pose2d setPos = drive.getPoseEstimate();

        //double leftIntegralSum = 0;
        //double rightIntegralSum = 0;
        //double lastLeftError = 0;
        //double lastRightError = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("leftHorizontal", leftHorizontal.getCurrentPosition());
            telemetry.addData("rightHorizontal", rightHorizontal.getCurrentPosition());
            telemetry.addData("leftVertical", leftVertical.getCurrentPosition());
            telemetry.addData("rightVertical", rightVertical.getCurrentPosition());
            telemetry.addData("isLocked", isLocked);
            telemetry.addData("Speed", speed);
            telemetry.update();

            // Drive train control

            if (!drivingMode){
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (gamepad1.guide){
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                }
                double botHeading = drive.getRawExternalHeading();
                double rotX = (x * Math.cos(-botHeading)) - (y * Math.sin(-botHeading));
                double rotY = (x * Math.sin(-botHeading)) + (y * Math.cos(-botHeading));
                rotX = rotX * 1.1;
                double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double leftFrontPower = (rotY + rotX + rx) / denom;
                double leftBackPower = (rotY - rotX + rx) / denom;
                double rightFrontPower = (rotY - rotX - rx) / denom;
                double rightBackPower = (rotY + rotX - rx) / denom;
                if (isLocked && leftFrontPower == 0.0 && rightFrontPower == 0.0 && leftBackPower == 0.0 && rightBackPower == 0.0) {
                    lockTo(drive, setPos);
                    drive.update();
                } else{
                    leftFrontDrive.setPower(leftFrontPower * speed);
                    rightFrontDrive.setPower(rightFrontPower * speed);
                    leftBackDrive.setPower(leftBackPower * speed);
                    rightBackDrive.setPower(rightBackPower * speed);
                    setPos = drive.getPoseEstimate();
                }
            } else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double leftFrontPower = (y + x + rx) / denom;
                double rightFrontPower = y - x - rx / denom;
                double leftBackPower = y - x + rx / denom;
                double rightBackPower = y + x - rx / denom;

                if (isLocked && leftFrontPower == 0.0 && rightFrontPower == 0.0 && leftBackPower == 0.0 && rightBackPower == 0.0) {
                    lockTo(drive, setPos);
                    drive.update();
                }
                else{
                    leftFrontDrive.setPower(leftFrontPower * speed);
                    rightFrontDrive.setPower(rightFrontPower * speed);
                    leftBackDrive.setPower(leftBackPower * speed);
                    rightBackDrive.setPower(rightBackPower * speed);
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    setPos = drive.getPoseEstimate();
                }
            }

            // Linear slide control
            float motorHorizontalButton = gamepad1.left_trigger;
            boolean motorHorizontalRetract = gamepad1.left_bumper;
            float motorVerticalButton = gamepad1.right_trigger;
            boolean motorVerticalRetract = gamepad1.right_bumper;
            boolean motorHorizontalFullExtension = gamepad1.dpad_up;
            boolean motorHorizontalFullRetraction = gamepad1.dpad_down;
            boolean motorVerticalFullExtension = gamepad1.y;
            boolean motorVerticalFullRetraction = gamepad1.a;
            boolean lockDriveTrain = gamepad1.left_stick_button;
            boolean speedButton = gamepad1.back;
            boolean turn180 = gamepad1.right_stick_button;
            boolean styleSwitchButton = gamepad1.start;

            if (lockDriveTrain && !pressedLastIteration){
                isLocked = !isLocked;
                if (isLocked){
                    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    drive.update();
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    setPos = drive.getPoseEstimate();
                }
            }
            pressedLastIteration = lockDriveTrain;




            if (styleSwitchButton && !pressedStyleSwitchLastIteration){
                drivingMode = !drivingMode;
            }
            pressedStyleSwitchLastIteration = styleSwitchButton;
          /*
          boolean isTurning = false;
          if (turn180 && !pressed180LastIteration) {
              isTurning = !isTurning;
              turnTo = drive.getExternalHeading();
              turnToMax = drive.getExternalHeading() + Math.toRadians(5);
              turnToMin = drive.getExternalHeading() - Math.toRadians(5);
              if (turnToMax + Math.toRadians(180) <= Math.toRadians(360)) {
                  turnToMax += Math.toRadians(180);
              }
              else {
                  turnToMax += Math.toRadians(180) - Math.toRadians(360);
              }
              if (turnToMin + Math.toRadians(180) <= Math.toRadians(360)) {
                  turnToMin += Math.toRadians(180);
              }
              else {
                  turnToMin += Math.toRadians(180) - Math.toRadians(360);
              }
          }
          pressed180LastIteration = turn180;
          double tempHeading = drive.getExternalHeading();
          telemetry.addData("Heading", tempHeading);
          if (isTurning){
              List<Double> temp = drive.getWheelPositions();
              lockTo(drive, new Pose2d(temp.get(0), temp.get(1), turnTo));
              //comment start
              if (tempHeading > turnToMin){
                  leftFrontDrive.setPower(1);
                  rightFrontDrive.setPower(-1);
                  leftBackDrive.setPower(1);
                  rightBackDrive.setPower(-1);
              }
              else if (tempHeading < turnToMax){
                  leftFrontDrive.setPower(-1);
                  rightFrontDrive.setPower(1);
                  leftBackDrive.setPower(-1);
                  rightBackDrive.setPower(1);
              }
              else {
                  isTurning = false;
              }
              //comment end
          }
          */
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
            float leftVerticalPosition = leftVertical.getCurrentPosition();
            float rightVerticalPosition = rightVertical.getCurrentPosition();
            if ((motorHorizontalButton != 0) && (leftHorizontalPosition <= 2900) && (rightHorizontalPosition >= -2900)) {
                leftHorizontal.setPower(speed * motorHorizontalButton);
                rightHorizontal.setPower(speed * -motorHorizontalButton);
            } else if ((motorHorizontalRetract) && ((leftHorizontalPosition >= 100) || leftHorizontalPosition <= -100) && (rightHorizontalPosition >= 100 || rightHorizontalPosition <= -100)) {
                leftHorizontal.setPower(-speed);
                rightHorizontal.setPower(speed);
            } else if (motorHorizontalFullExtension){
                isFullHorizontalExtent = true;
                leftHorizontal.setPower(1.5);
                rightHorizontal.setPower(-1.5);
            } else if (motorHorizontalFullRetraction) {
                isFullHorizontalRetract = true;
                leftHorizontal.setPower(-1.5);
                rightHorizontal.setPower(1.5);
            } else if (!(isFullHorizontalExtent || isFullHorizontalRetract)) {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
            } else if (isFullHorizontalExtent && !(leftHorizontal.getCurrentPosition() <= 2900 && rightHorizontal.getCurrentPosition() >= -2900)){
                isFullHorizontalExtent = false;
            } else if (isFullHorizontalRetract && !(((leftHorizontalPosition >= 50) || leftHorizontalPosition <= -50) && (rightHorizontalPosition >= 50 || rightHorizontalPosition <= -50))){
                isFullHorizontalRetract = false;
                leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if ((motorVerticalButton != 0) && (leftVerticalPosition <= 2900) && (rightVerticalPosition >= -2900)) {
                leftVertical.setPower(speed * motorVerticalButton);
                rightVertical.setPower(speed * -motorVerticalButton);
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
            } else if ((motorVerticalRetract) && ((leftVerticalPosition >= 100) || leftVerticalPosition <= -100) && (rightVerticalPosition >= 100 || rightVerticalPosition <= -100)) {
                leftVertical.setPower(-speed);
                rightVertical.setPower(speed);
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
            } else if (motorVerticalFullExtension){
                isFullVerticalExtent = true;
                leftVertical.setPower(1.5);
                rightVertical.setPower(-1.5);
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
            } else if (motorVerticalFullRetraction) {
                isFullVerticalRetract = true;
                leftVertical.setPower(-1.5);
                rightVertical.setPower(1.5);
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
            } else if (!(isFullVerticalExtent || isFullVerticalRetract)) {
                leftVertical.setPower(0);
                rightVertical.setPower(0);
              /*
              double leftError = leftVerticalStandbyPos - leftVertical.getCurrentPosition();
              double rightError = rightVerticalStandbyPos - rightVertical.getCurrentPosition();
              double leftDerivative = (leftError - lastLeftError) / runtime.seconds();
              double rightDerivative = (rightError - lastRightError) / runtime.seconds();
              leftIntegralSum = leftIntegralSum + (leftError * runtime.seconds());
              rightIntegralSum = rightIntegralSum + (rightError * runtime.seconds());
              double leftOut = (Kp * leftError) + (Ki * leftIntegralSum) + (Kd * leftDerivative);
              double rightOut = (Kp * rightError) + (Ki * rightIntegralSum) + (Kd * rightDerivative);
              leftVertical.setPower(leftOut);
              rightVertical.setPower(-leftOut);
              lastLeftError = leftError;
              lastRightError = rightError;
               */
            } else if (isFullVerticalExtent && !(leftVertical.getCurrentPosition() <= 2900 && rightVertical.getCurrentPosition() >= -2900)){
                isFullVerticalExtent = false;
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
            } else if (isFullVerticalRetract && !(((leftVerticalPosition >= 50) || leftVerticalPosition <= -50) && (rightVerticalPosition >= 50 || rightVerticalPosition <= -50))) {
                isFullVerticalRetract = false;
                leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftVerticalStandbyPos = leftVertical.getCurrentPosition();
                rightVerticalStandbyPos = rightVertical.getCurrentPosition();
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
    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * runtime.seconds();
        double derivative = (error - lastError) / runtime.seconds();
        lastError = error;
        runtime.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    //tune Kp, Kd, Ki; call function: motor.setPower(PIDControl(motorRefPosition, motor.getCurrentPosition()))
}




