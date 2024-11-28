package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Individual Slide Debug", group = "Test")
public class vertslideencoderdebug extends LinearOpMode {

    private DcMotor leftVertical;
    private DcMotor rightVertical;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftVertical = hardwareMap.get(DcMotor.class, "leftVerticalSlide");
        rightVertical = hardwareMap.get(DcMotor.class, "rightVerticalSlide");

        // Configure motors
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
            // Control left slide with Left Bumper
            if (gamepad1.left_bumper) {
                leftVertical.setPower(2.0); // Move up
            } else if (gamepad1.left_trigger > 0.2) {
                leftVertical.setPower(-2.0); // Move down
            } else {
                leftVertical.setPower(0); // Stop
            }

            // Control right slide with Right Bumper
            if (gamepad1.right_bumper) {
                rightVertical.setPower(2.0); // Move up
            } else if (gamepad1.right_trigger > 0.2) {
                rightVertical.setPower(-2.0); // Move down
            } else {
                rightVertical.setPower(0); // Stop
            }

            // Log encoder positions and motor power to telemetry
            telemetry.addData("Left Slide Position", leftVertical.getCurrentPosition());
            telemetry.addData("Right Slide Position", rightVertical.getCurrentPosition());
            telemetry.addData("Left Motor Power", leftVertical.getPower());
            telemetry.addData("Right Motor Power", rightVertical.getPower());
            telemetry.update();
        }
    }
}
