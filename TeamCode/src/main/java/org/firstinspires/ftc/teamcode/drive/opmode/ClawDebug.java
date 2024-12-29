package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Test", group = "Test")
public class ClawDebug extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the servo
        Servo outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
        Servo outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
        outtakeClawLeft.scaleRange(0, 0.45);
        outtakeClawRight.scaleRange(0, 0.45);
        outtakeClawLeft.setDirection(Servo.Direction.FORWARD);
        outtakeClawRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                outtakeClawLeft.setPosition(0);
                outtakeClawRight.setPosition(0);
            } else if (gamepad1.b) {
                outtakeClawLeft.setPosition(0.5);
                outtakeClawRight.setPosition(0.5);
            }

            telemetry.addData("Claw Left", outtakeClawLeft.getPosition());
            telemetry.addData("Claw Right", outtakeClawRight.getPosition());
            telemetry.update();
        }
    }
}
