package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class SpintakeTest extends LinearOpMode{

    CRServo spintakeLeft;
    CRServo spintakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        spintakeLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");
        spintakeRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        spintakeLeft.setDirection(CRServo.Direction.FORWARD);
        spintakeRight.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if(gamepad1.a) {
                spintakeLeft.setPower(1);
                spintakeRight.setPower(1);
            }
            else if (gamepad1.b) {
                spintakeLeft.setPower(-1);
                spintakeRight.setPower(-1);
            }
            else {
                spintakeLeft.setPower(0);
                spintakeRight.setPower(0);
            }

        }
    }
}
