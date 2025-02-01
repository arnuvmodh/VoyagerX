package org.firstinspires.ftc.teamcode.subsystems.gobuilda;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpinTake {
    private final CRServo _left;
    private final CRServo _right;

    public SpinTake(HardwareMap hardwareMap, String left, String right) {
        _left = hardwareMap.get(CRServo.class, left);
        _right = hardwareMap.get(CRServo.class, right);
        _left.setDirection(CRServo.Direction.FORWARD);
        _right.setDirection(CRServo.Direction.REVERSE);
    }

    public void spinIn(double power) {
        _left.setPower(-power);
        _right.setPower(-power);
    }

    public void spinOut(double power) {
        _left.setPower(power);
        _right.setPower(power);
    }

    public void stop() {
        _left.setPower(0);
        _right.setPower(0);
    }

}