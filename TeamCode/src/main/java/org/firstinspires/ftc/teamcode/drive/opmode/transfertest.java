package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Transfer Test", group = "Test")
public class transfertest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.intakePivot.flipFront();
            } else if (gamepad1.b) {
                robot.intakePivot.flipBack();
            }

            // Log encoder positions and motor power to telemetry
            telemetry.addData("Left Pivot Position", robot.intakePivot.getLeftPosition());
            telemetry.addData("Right Pivot Position", robot.intakePivot.getRightPosition());
            telemetry.update();
        }
    }
}
