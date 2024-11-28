package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Run To Position Test", group = "Test")
public class runtopositiontest extends LinearOpMode {

    private DcMotor leftVertical;
    private DcMotor rightVertical;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftVertical = hardwareMap.get(DcMotor.class, "leftVerticalSlide");
        rightVertical = hardwareMap.get(DcMotor.class, "rightVerticalSlide");

        // Reset encoders and configure motors
        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Move slides to a target position when A is pressed
                moveToPosition(2000, 1);
            } else if (gamepad1.b) {
                // Move slides back to a lower position when B is pressed
                moveToPosition(0, 1);
            }

            // Display current positions on telemetry
            telemetry.addData("Left Position", leftVertical.getCurrentPosition());
            telemetry.addData("Right Position", rightVertical.getCurrentPosition());
            telemetry.update();
        }
    }

    private void moveToPosition(int targetPosition, double power) {
        // Set target positions
        leftVertical.setTargetPosition(targetPosition); // Reverse direction for left slide
        rightVertical.setTargetPosition(targetPosition);

        // Set motors to RUN_TO_POSITION mode
        leftVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for both motors
        leftVertical.setPower(Math.abs(power));
        rightVertical.setPower(Math.abs(power));

        // Wait for motors to reach target
        while (opModeIsActive() && (leftVertical.isBusy() || rightVertical.isBusy())) {
            telemetry.addData("Left Target", leftVertical.getTargetPosition());
            telemetry.addData("Right Target", rightVertical.getTargetPosition());
            telemetry.addData("Left Position", leftVertical.getCurrentPosition());
            telemetry.addData("Right Position", rightVertical.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors once target is reached
        leftVertical.setPower(0);
        rightVertical.setPower(0);

        // Reset to RUN_USING_ENCODER for future manual control
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
