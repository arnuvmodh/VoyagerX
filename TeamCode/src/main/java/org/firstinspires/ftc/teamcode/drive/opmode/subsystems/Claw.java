package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    private final Servo _left;
    private final Servo _right;

    public Claw(HardwareMap hardwareMap, String left, String right, double min, double max) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, right);
        _left.setDirection(Servo.Direction.FORWARD);
        _right.setDirection(Servo.Direction.REVERSE);
        _left.scaleRange(min, max);
        _right.scaleRange(min, max);
    }

    public Claw(HardwareMap hardwareMap, String left, double min, double max) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, left);
        _left.setDirection(Servo.Direction.FORWARD);
        _right.setDirection(Servo.Direction.FORWARD);
        _left.scaleRange(min, max);
        _right.scaleRange(min, max);
    }

    public void open() {
        _left.setPosition(0);
        _right.setPosition(0);
    }

    public void close() {
        _left.setPosition(1);
        _right.setPosition(1);
    }

    public void openTo(double position) {
        _left.setPosition(position);
        _right.setPosition(position);
    }

    public boolean isOpen() {
        return (_left.getPosition()<=(0.01) &&
                _right.getPosition()<=(0.01));
    }

    public boolean isClosed() {
        return (_left.getPosition()>=(0.99) &&
                _right.getPosition()>=(0.99));
    }

    public boolean isAt(double position) {
        return (_left.getPosition()>=(position-0.01) && _left.getPosition()<=(position+0.01) &&
                _right.getPosition()>=(position-0.01) && _right.getPosition()<=(position+0.01));
    }

    public double getLeftPosition() {
        return _left.getPosition();
    }

    public double getRightPosition() {
        return _right.getPosition();
    }
}