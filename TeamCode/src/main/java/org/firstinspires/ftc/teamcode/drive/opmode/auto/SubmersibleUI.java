package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SubmersibleUI {
    private final int _id;
    private double _x = 0;
    private double _y = 0;
    private double _failStrafe = 1;
    private double increment = 3;

    public SubmersibleUI(int id) {
        _id = id;
    }

    private boolean upDown = false, downDown = false, leftDown = false, rightDown = false;
    private boolean aDown = false, bDown = false, xDown = false, yDown = false;
    boolean userInput(Gamepad gamepad, Telemetry telemetry) {
        telemetry.addData("Submersible Sample ID", _id);
        telemetry.addData("Increment", increment);
        telemetry.addData("X Position", _x);
        telemetry.addData("Intake Slide Position", _y);
        telemetry.addData("Fail Strafe", _failStrafe);
        telemetry.addLine("X - Confirm");
        telemetry.addLine("O - Fail Strafe");
        telemetry.addLine("□ - Decrease Increment");
        telemetry.addLine("△ - Increase Increment");
        telemetry.update();

        if(gamepad.dpad_up && !upDown) _y+=(increment/10);
        upDown = gamepad.dpad_up;
        if(gamepad.dpad_down && !downDown) _y-=(increment/10);
        downDown = gamepad.dpad_down;
        if(gamepad.dpad_left && !leftDown) _x-=increment;
        leftDown = gamepad.dpad_left;
        if(gamepad.dpad_right && !rightDown) _x+=increment;
        rightDown = gamepad.dpad_right;

        if(gamepad.b && !bDown) {
            _failStrafe = -1*_failStrafe;
        }
        bDown = gamepad.b;

        if(gamepad.x && !xDown) {
            increment-=0.5;
        }
        xDown = gamepad.x;
        if(gamepad.y && !yDown) {
            increment+=0.5;
        }
        yDown = gamepad.y;

        if(gamepad.a && !aDown) {
            return true;
        }
        aDown = gamepad.a;
        return false;
    }

    Pose2d getTrajectory() {
        return new Pose2d(17.5, 60+_x, 6.2498);
    }

    double getIntakeSlidePosition() {
        return _y;
    }

    double getFailStrafe() {
        return _failStrafe;
    }


}
