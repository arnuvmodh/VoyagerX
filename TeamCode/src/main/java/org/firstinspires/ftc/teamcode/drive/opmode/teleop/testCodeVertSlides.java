    package org.firstinspires.ftc.teamcode.drive.opmode.teleop;


    import com.acmerobotics.roadrunner.geometry.Pose2d;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
    import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;


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
        private Servo servoLeft = null;
        private Servo servoRight = null;
        private Servo servoPivot = null;
        private Servo servoClawLeft = null;
        private Servo servoClawRight = null;
        private Servo outtakePivotLeft;
        private Servo outtakePivotRight;
        private Servo outtakeClawLeft;
        private Servo outtakeClawRight;
        private Servo servoHang;

        // Declare OpMode members for the linear slides
        private Servo leftHorizontal = null;
        private Servo rightHorizontal = null;
        private DcMotor leftVertical = null;
        private DcMotor rightVertical = null;

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

            servoLeft = hardwareMap.get(Servo.class, "intakePivotLeft");
            servoRight = hardwareMap.get(Servo.class, "intakePivotRight");
            servoRight.setDirection(Servo.Direction.REVERSE);
            servoLeft.setDirection(Servo.Direction.FORWARD);
            servoLeft.scaleRange(0.205, 0.8);
            servoRight.scaleRange(0.205, 0.8);
            servoLeft.setPosition(0);
            servoRight.setPosition(0);
            double posFieldEdge = 0.75;
            double posUnderBar = 0.85;

            double pivotServoPosition = 0.5;
            servoPivot = hardwareMap.get(Servo.class, "servoPivot");
            servoPivot.setDirection(Servo.Direction.FORWARD);
            servoPivot.scaleRange(0, 1);
            servoPivot.setPosition(pivotServoPosition);

            servoClawLeft = hardwareMap.get(Servo.class, "intakeClawLeft");
            servoClawRight = hardwareMap.get(Servo.class, "intakeClawRight");
            servoClawLeft.setDirection(Servo.Direction.FORWARD);
            servoClawRight.setDirection(Servo.Direction.REVERSE);
            servoClawLeft.setPosition(0.575);
            servoClawRight.setPosition(0.575);

            servoHang = hardwareMap.get(Servo.class, "servoHang");
            servoHang.setDirection(Servo.Direction.REVERSE);
            servoHang.setPosition(0);

            outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
            outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
            outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
            outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
            outtakeClawRight.setDirection(Servo.Direction.REVERSE);
            outtakePivotRight.setDirection(Servo.Direction.REVERSE);
            outtakeClawLeft.scaleRange(0, 0.45);
            outtakeClawRight.scaleRange(0, 0.45);
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
            leftHorizontal = hardwareMap.get(Servo.class, "leftHorizontalSlide");
            leftHorizontal.setDirection(Servo.Direction.FORWARD);
            rightHorizontal = hardwareMap.get(Servo.class, "rightHorizontalSlide");
            rightHorizontal.setDirection(Servo.Direction.REVERSE);
            leftVertical = hardwareMap.get(DcMotor.class, "leftVerticalSlide");
            leftVertical.setDirection(DcMotorSimple.Direction.FORWARD);
            rightVertical = hardwareMap.get(DcMotor.class, "rightVerticalSlide");
            rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);


            // Set horizontal slides
            leftHorizontal.scaleRange(0, 0.45);
            rightHorizontal.scaleRange(0, 0.45);
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
            boolean isFullVerticalRetract = false;
            boolean isFullVerticalExtent = false;
            boolean drivingMode = false;
            //false is field centric, true is robot centric
            boolean pressedLastIteration = false;
            boolean pressedSpeedLastIteration = false;
            boolean pressedPivotServoLastIteration = false;
            boolean pressedPickServoLastIteration = false;
            boolean pressedHangLastIteration = false;
            boolean pickServoSwitch = false;
            double speed = 1.0;
            boolean pressedOuttakeClawLastIteration = false;
            boolean pressedOuttakePivotLastIteration = false;
            boolean pressedSpecimenIntakeLastIteration = false;
            boolean outtakeClawPosition = false;
            boolean outtakePivotPosition = false;
            double timeForTransfer = 0;
            boolean waitForClose = false;
            boolean hangToggle = false;
            Pose2d setPos = drive.getPoseEstimate();
            boolean canDoSpecimen = true;
            boolean specimenToggle = false;
            boolean goingDown = true;

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
                telemetry.addData("leftVertical", leftVertical.getCurrentPosition());
                telemetry.addData("rightVertical", rightVertical.getCurrentPosition());
                telemetry.addData("Speed", speed);
                telemetry.addData("Driving Mode (false = fieldcentric, true=robotcentric)", drivingMode);
                telemetry.addData("Heading (degrees)", Math.toDegrees(botHeading));
                telemetry.addData("Bot Position (poseEstimate)", drive.getPoseEstimate().toString());
                telemetry.update();


                // Drive train control
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;


                if (gamepad2.a) {
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


                leftFrontDrive.setPower(leftFrontPower * speed);
                rightFrontDrive.setPower(rightFrontPower * speed);
                leftBackDrive.setPower(leftBackPower * speed);
                rightBackDrive.setPower(rightBackPower * speed);
                setPos = drive.getPoseEstimate();
                runtime.reset();

                if (runtime.seconds() > 0.8 && runtime.seconds() < 1) {
                    setPos = drive.getPoseEstimate();
                }

                // Controls
//                boolean motorHorizontalButton = (gamepad1.left_trigger > 0.2);
                boolean specimenSlidePosition = (gamepad1.left_trigger > 0.2);
                boolean motorHorizontalRetract = gamepad1.left_bumper;
                boolean motorHorizontalFullExtension = gamepad1.dpad_up;
                boolean motorHorizontalFullRetraction = gamepad2.touchpad || gamepad1.dpad_down;
                float motorVerticalButton = gamepad1.right_trigger;
                boolean motorVerticalRetract = gamepad1.right_bumper;
                boolean motorVerticalFullExtension = gamepad1.y;
                boolean motorVerticalFullRetraction = (gamepad2.left_trigger > 0.2)||gamepad1.a;
                boolean speedButton = gamepad2.left_stick_button;
                boolean servoOut = (gamepad2.right_trigger > 0.2)||(gamepad1.dpad_left);
                boolean servoIn = (gamepad1.dpad_right || gamepad2.touchpad);
                boolean servoPivotRight = gamepad2.dpad_right;
                boolean servoPivotLeft = gamepad2.dpad_left;
                boolean servoPivotReset = gamepad2.dpad_down;
                boolean servoPickButton = gamepad2.dpad_up;
                boolean fieldEdgeButton = gamepad2.left_bumper;
                boolean underBarButton = gamepad2.right_bumper;
                boolean outtakeClawButton = gamepad2.y;
                boolean outtakePivotButton = gamepad2.x;
                boolean highBucketButton = gamepad2.b;
                boolean specimenGrabButton = gamepad2.back;
                boolean hangButton = gamepad2.right_stick_button;

                if(hangButton && !pressedHangLastIteration) {
                    servoHang.setPosition(0.5);
                }
                /*else {
                    servoHang.setPosition(0);
                }*/
                pressedHangLastIteration = hangButton;

                if(specimenGrabButton && !pressedSpecimenIntakeLastIteration){
                    if (!outtakePivotPosition){
                        outtakePivotLeft.setPosition(0.19);
                        outtakePivotRight.setPosition(0.20);
                        outtakePivotPosition = true;
                    } else {
                        outtakePivotLeft.setPosition(0.975);
                        outtakePivotRight.setPosition(1);
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
                    pivotServoPosition = 0.5;
                    servoPivot.setPosition(pivotServoPosition);
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



                if (speedButton && !pressedSpeedLastIteration){
                    if (speed > 0.9){
                        speed /= 3;
                    }
                    else{
                        speed = 1.0;
                    }
                }
                pressedSpeedLastIteration = speedButton;

                // Horizontal slides
//                if (horizontalSlidePosition < 0.95 && motorHorizontalButton) {
//                    horizontalSlidePosition += 0.1;
//                } else if (horizontalSlidePosition > 0.05 && motorHorizontalRetract) {
//                    horizontalSlidePosition -= 0.1;
                if (horizontalSlidePosition > 0.05 && motorHorizontalRetract) {
                    horizontalSlidePosition -= 0.1;
                } else if (motorHorizontalFullExtension) {
                    horizontalSlidePosition = 1;
                } else if (motorHorizontalFullRetraction) {
                    horizontalSlidePosition = 0;
                }
                leftHorizontal.setPosition(horizontalSlidePosition);
                rightHorizontal.setPosition(horizontalSlidePosition);

                // Vertical Slides
                float leftVerticalPosition = -leftVertical.getCurrentPosition();
                float rightVerticalPosition = -rightVertical.getCurrentPosition();
                if ((motorVerticalButton != 0) && (leftVerticalPosition <= 3050) && (rightVerticalPosition >= -3050)) {
                    leftVertical.setPower(speed * motorVerticalButton);
                    rightVertical.setPower(speed * -motorVerticalButton);
                    canDoSpecimen = false;
                } else if ((motorVerticalRetract) && ((leftVerticalPosition >= 50) || leftVerticalPosition <= -50) && (rightVerticalPosition >= 50 || rightVerticalPosition <= -50)) {
                    leftVertical.setPower(-speed);
                    rightVertical.setPower(speed);
                    canDoSpecimen = false;
                } else if (motorVerticalFullExtension){
                    isFullVerticalExtent = true;
                    leftVertical.setPower(1.5);
                    rightVertical.setPower(-1.5);
                    canDoSpecimen = false;
                } else if (motorVerticalFullRetraction) {
                    isFullVerticalRetract = true;
                    leftVertical.setPower(-1.5);
                    rightVertical.setPower(1.5);
                    canDoSpecimen = false;
                } else if (!(isFullVerticalExtent || isFullVerticalRetract)) {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                    canDoSpecimen = true;
                } else if (isFullVerticalExtent && !(leftVerticalPosition <= 3050 && rightVerticalPosition >= -3050)){
                    isFullVerticalExtent = false;
                    canDoSpecimen = true;
                } else if (isFullVerticalRetract && !(((leftVerticalPosition >= 50) || leftVerticalPosition <= -50) && (rightVerticalPosition >= 50 || rightVerticalPosition <= -50))) {
                    isFullVerticalRetract = false;
                    canDoSpecimen = true;
                    leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                if (canDoSpecimen && specimenSlidePosition) {
                    specimenToggle = true;
                    if (leftVerticalPosition > 1930 || rightVerticalPosition < -1930) {
                        leftVertical.setPower(-1);
                        rightVertical.setPower(1);
                        goingDown = true;
                    } else {
                        leftVertical.setPower(1);
                        rightVertical.setPower(-1);
                        goingDown = false;
                    }
                }
                if (specimenToggle) {
                    if (goingDown && (leftVerticalPosition < 1930 && rightVerticalPosition > -1930)) {
                        leftVertical.setPower(0);
                        rightVertical.setPower(0);
                        specimenToggle = false;
                    } else if (!goingDown && (leftVerticalPosition > 1930 && rightVerticalPosition < -1930)) {
                        leftVertical.setPower(0);
                        rightVertical.setPower(0);
                        specimenToggle = false;
                    }
                }

                // specimenSlidePosition 1930

//                if (horizontalSlidePosition < 0.95 && motorHorizontalButton) {
//                    horizontalSlidePosition += 0.1;
//                } else if (horizontalSlidePosition > 0.05 && motorHorizontalRetract) {
//                    horizontalSlidePosition -= 0.1;
                if (horizontalSlidePosition > 0.05 && motorHorizontalRetract) {
                    horizontalSlidePosition -= 0.1;
                } else if (motorHorizontalFullExtension) {
                    horizontalSlidePosition = 1;
                } else if (motorHorizontalFullRetraction) {
                    horizontalSlidePosition = 0;
                }
                leftHorizontal.setPosition(horizontalSlidePosition);
                rightHorizontal.setPosition(horizontalSlidePosition);

            }
        }
    }