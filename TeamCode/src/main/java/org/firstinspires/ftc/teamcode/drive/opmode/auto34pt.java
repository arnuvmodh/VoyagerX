package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@Disabled()
public class auto34pt extends LinearOpMode {
    enum State {
        traj1,
        traj2,
        traj3,
        traj4,
        traj5,
        traj6,
        traj7,
        traj8,
        idle
    }


    private Servo servoLeft;
    private Servo servoRight;
    private Servo servoPivot;
    private Servo servoClawLeft;
    private Servo servoClawRight;
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClawLeft;
    private Servo outtakeClawRight;

    // Declare OpMode members for the linear slides
    private DcMotor leftHorizontal;
    private DcMotor rightHorizontal;
    private DcMotor leftVertical;
    private DcMotor rightVertical;
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException{
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoLeft.scaleRange(0, 0.625);
        servoRight.scaleRange(0, 0.625);
        servoLeft.setPosition(0);
        servoRight.setPosition(0);

        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        servoPivot.setDirection(Servo.Direction.FORWARD);
        servoPivot.scaleRange(0, 1);
        servoPivot.setPosition(0.5);

        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");
        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.REVERSE);
        servoClawLeft.setPosition(0.3);
        servoClawRight.setPosition(0.3);

        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
        outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
        outtakeClawRight.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight.setDirection(Servo.Direction.REVERSE);
        outtakeClawLeft.scaleRange(0, 0.55);
        outtakeClawRight.scaleRange(0, 0.55);
        outtakeClawRight.setPosition(1);
        outtakeClawLeft.setPosition(1);
        outtakePivotLeft.setPosition(0.975);
        outtakePivotRight.setPosition(1);

        // Initialize the hardware variables for the linear slides
        leftHorizontal = hardwareMap.get(DcMotor.class, "left_slide");
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontal = hardwareMap.get(DcMotor.class, "right_slide");
        rightHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set up encoders for horizontal slides
        leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0, 0));

        State curState = State.idle;


        //IMPLEMENT THESE TRAJECTORY SEQUENCES ASYNCHRONOUSLY TO FOLLOW AUTO PAT
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-23, 4, 0.9))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-13.6212, 7.5, 1.4318))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-24, 3.5, 0.9))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-17.8625, 7.3, 1.6415))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-24, 3.25, 0.9))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-22.2, 13.47, 1.9))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-24, 3.1, 0.9))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-13.6212, 7.5, (Math.PI/2)))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(traj1);
        curState = State.traj1;
        boolean riseVertSlides = true;
        boolean lowerVertSlides = false;
        boolean extendHorSlides = false;
        boolean edgingHorSlides = false;
        boolean retractHorSlides = false;

        boolean[] oneTimeSwitch = new boolean[100];
        for (int i = 0; i < 100; i++) {
            oneTimeSwitch[i] = true;
        }

        int[] verticalPositions = new int[] {-3200, -3400, -3500, -3500, -3700};
        int i = 0;
        timer.reset();
        while (opModeIsActive()) {
            if(riseVertSlides  && lowerVertSlides){
                lowerVertSlides= false;
            }
            switch (curState) {
                case idle:
                    break;
                case traj1:
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        oneTimeSwitch[0] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj2);
                        curState = State.traj2;
                        i += 1;
                        timer.reset();

                    }
                    break;
                case traj2:
                    if (oneTimeSwitch[1] && timer.seconds() > 0.5) {
                        lowerVertSlides = true;
                        extendHorSlides = true;
                        oneTimeSwitch[1] = false;
                    }
                    if(timer.seconds() > 1.6){
                        intakeGrab();
                    }

                    if(oneTimeSwitch[2] && timer.seconds() > 2){
                        retractHorSlides = true;
                        oneTimeSwitch[2] = false;
                        intakeFlipIn();
                    }
                    if(timer.seconds() > 3.3){
                        outtakeGrab();
                    }
                    if(timer.seconds() > 3.8){
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 4.2) {
                        drive.followTrajectoryAsync(traj3);
                        curState = State.traj3;
                        timer.reset();
                    }
                    break;
                case traj3:
                    if(oneTimeSwitch[3]){
                        riseVertSlides = true;
                        oneTimeSwitch[3] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 3.5) {
                        oneTimeSwitch[4] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj4);
                        curState = State.traj4;
                        timer.reset();
                        i += 1;
                    }

                    break;
                case traj4:
                    if (oneTimeSwitch[5] && timer.seconds() > 0.8) {
                        lowerVertSlides = true;
                        extendHorSlides = true;
                        oneTimeSwitch[5] = false;
                    }
                    if(timer.seconds() > 2){
                        intakeGrab();
                    }

                    if(oneTimeSwitch[6] && timer.seconds() > 2.4){
                        retractHorSlides = true;
                        oneTimeSwitch[6] = false;
                        intakeFlipIn();
                    }
                    if(timer.seconds() > 3.7){
                        outtakeGrab();
                    }
                    if(timer.seconds() > 4.2){
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 4.6) {
                        drive.followTrajectoryAsync(traj5);
                        curState = State.traj5;
                        timer.reset();
                    }
                    break;
                case traj5:
                    if(oneTimeSwitch[7]){
                        riseVertSlides = true;
                        oneTimeSwitch[7] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        oneTimeSwitch[8] = false;
                        intakeFlipOut();
                        drive.followTrajectoryAsync(traj6);
                        curState = State.traj6;
                        timer.reset();
                        i += 1;
                    }

                    break;
                case traj6:
                    if (oneTimeSwitch[9] && timer.seconds() > 0.8) {
                        lowerVertSlides = true;
                        extendHorSlides = true;
                        edgingHorSlides = true;
                        servoPivot.setPosition(0.45);
                        oneTimeSwitch[9] = false;
                    }

                    if (timer.seconds() > 2.75 && oneTimeSwitch[10]) {
                        oneTimeSwitch[10] = false;
                        servoPivot.setPosition(0);
                    }

                    if(timer.seconds() > 3 && oneTimeSwitch[11]){
                        oneTimeSwitch[11] = false;
                        intakeEdgeClawPosition();
                    }

                    if (timer.seconds() > 3.5 && oneTimeSwitch[12]) {
                        oneTimeSwitch[12] = false;
                        intakeGrab();
                    }

                    if(oneTimeSwitch[13] && timer.seconds() > 4){
                        retractHorSlides = true;
                        oneTimeSwitch[13] = false;
                        servoPivot.setPosition(0.5);
                        intakeFlipIn();
                    }
                    if(timer.seconds() > 5.3){
                        outtakeGrab();
                    }
                    if(timer.seconds() > 5.8){
                        intakeLetGo();
                    }
                    if (!drive.isBusy() && timer.seconds() > 6.2) {
                        drive.followTrajectoryAsync(traj7);
                        curState = State.traj7;
                        timer.reset();
                    }

                    break;
                case traj7:
                    if(oneTimeSwitch[14]){
                        i += 1;
                        riseVertSlides = true;
                        oneTimeSwitch[14] = false;
                    }
                    scoreBasket(timer);
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        drive.followTrajectoryAsync(traj8);
                        curState = State.traj8;
                        timer.reset();
                    }
                    break;
                case traj8:
                    if(timer.seconds()>.1) {
                       intakeGrab();
                    }
                    if(oneTimeSwitch[15]&&timer.seconds()>0.2) {
                        lowerVertSlides = true;
                        oneTimeSwitch[15] = false;
                    }
                    if (!drive.isBusy() && timer.seconds() > 3) {
                        curState = State.idle;
                    }
                    break;
            }

            if (!edgingHorSlides && leftHorizontal.getCurrentPosition() < 1050 && rightHorizontal.getCurrentPosition() < 1050 && extendHorSlides) {
                leftHorizontal.setPower(0.5);
                rightHorizontal.setPower(0.5);
            } else if (edgingHorSlides && leftHorizontal.getCurrentPosition() < 870 && rightHorizontal.getCurrentPosition() < 870 && extendHorSlides) {
                leftHorizontal.setPower(0.2);
                rightHorizontal.setPower(0.2);
            } else if (leftHorizontal.getCurrentPosition() > 0 && rightHorizontal.getCurrentPosition() > 0 && retractHorSlides) {
                leftHorizontal.setPower(-1);
                rightHorizontal.setPower(-1);
            } else {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
                extendHorSlides = false;
                retractHorSlides = false;
            }


            if (rightVertical.getCurrentPosition() > verticalPositions[i] && riseVertSlides) {
                leftVertical.setPower(1);
                rightVertical.setPower(1);
            } else if (rightVertical.getCurrentPosition() < 0 && lowerVertSlides) {
                leftVertical.setPower(-1);
                rightVertical.setPower(-1);
            } else {
                leftVertical.setPower(0);
                rightVertical.setPower(0);
                riseVertSlides = false;
                lowerVertSlides = false;
            }


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("riseVertSlides", riseVertSlides);
            telemetry.addData("lowerVertSlides", lowerVertSlides);
            telemetry.addData("rightVertical", rightVertical.getCurrentPosition());
            telemetry.addData("leftVertical", leftVertical.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


        }
    void scoreBasket(ElapsedTime timer){
        if(timer.seconds() > 1.6){
            outtakeFlipOut();
        }
        if(timer.seconds() > 2.1){
            outtakeLetGo();
        }
        if(timer.seconds()>2.3){
            outtakeFlipIn();
        }
    }

    void outtakeFlipOut() {
        outtakePivotLeft.setPosition(0.5);
        outtakePivotRight.setPosition(0.525);
    }


    void outtakeFlipIn() {
        outtakePivotLeft.setPosition(0.975);
        outtakePivotRight.setPosition(1);
    }

    void outtakeLetGo() {
        outtakeClawLeft.setPosition(0);
        outtakeClawRight.setPosition(0);
    }

    void outtakeGrab() {
        outtakeClawRight.setPosition(1);
        outtakeClawLeft.setPosition(1);
    }

    void intakeLetGo() {
        servoClawLeft.setPosition(0.3);
        servoClawRight.setPosition(0.3);
    }

    void intakeGrab() {
        servoClawLeft.setPosition(0.525);
        servoClawRight.setPosition(0.525);
    }

    void intakeEdgeClawPosition() {
        servoClawLeft.setPosition(0.4);
        servoClawRight.setPosition(0.35);
    }

    void intakeFlipOut() {
        servoLeft.setPosition(1);
        servoRight.setPosition(1);
    }

    void intakeFlipIn() {
        servoLeft.setPosition(0);
        servoRight.setPosition(0);
    }

}







