package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled()
public class vertSlideBrakeTest extends LinearOpMode {

    private DcMotor leftVertical;
    private DcMotor rightVertical;

    double integralSum = 0;

    ElapsedTime runtime = new ElapsedTime();
    double lastError = 0;
    double Kp = 0.1;
    double Kd = 0.05;
    double Ki = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        boolean state = false;

        int leftVertBrakePos = 0;
        int rightVertBrakePos = 0;

        waitForStart();
        if (isStopRequested()){
            return;
        }
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftVertical.setPower(-1);
                rightVertical.setPower(-1);
                state = false;
            } else if (gamepad1.right_bumper) {
                leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftVertical.setPower(1);
                rightVertical.setPower(1);
                state = false;
            } else {
                leftVertical.setPower(0);
                rightVertical.setPower(0);
                if (!state) {
                    leftVertBrakePos = leftVertical.getCurrentPosition();
                    rightVertBrakePos = rightVertical.getCurrentPosition();
                    state = true;
                }
            }

            if (state) {
                leftVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVertical.setTargetPosition(leftVertBrakePos);
                rightVertical.setTargetPosition(rightVertBrakePos);
                leftVertical.setPower(0.05);
                rightVertical.setPower(0.05);
                if(leftVertical.isBusy()) {
                    leftVertical.setPower(0.25);
                }
                if(rightVertical.isBusy()) {
                    rightVertical.setPower(0.25);
                }
            }
        }
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

    public double easing(double val, double targetval){
        double diff = val - targetval;
        boolean isNegga = false;
        double power;

        if (diff < 0) {
            isNegga = true;
            diff = Math.abs(diff);
        }
        if (diff >= 100) {
            if (isNegga){
                return -1;
            } else {
                return 1;
            }
        }
        power = (100 - diff) / 100;

        if (isNegga){
            return -power;
        } else {
            return power;
        }
    }
}