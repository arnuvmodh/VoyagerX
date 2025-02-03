package org.firstinspires.ftc.teamcode.subsystems.axon;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AxonPivot {
    private final Servo _left;
    private final Servo _right;
    private double _offset = 0;

    public AxonPivot(HardwareMap hardwareMap, String left, double min, double max) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, left);;
        _left.setDirection(Servo.Direction.FORWARD);
        _left.scaleRange(min, max);
    }

    public AxonPivot(HardwareMap hardwareMap, String left, String right, double min, double max) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, right);
        _left.setDirection(Servo.Direction.FORWARD);
        _right.setDirection(Servo.Direction.REVERSE);
        _left.scaleRange(min, max);
        _right.scaleRange(min, max);
    }

    // Offset added to left servo
    public AxonPivot(HardwareMap hardwareMap, String left, String right, double min, double max, double offset) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, right);
        _offset = offset;
        _left.setDirection(Servo.Direction.FORWARD);
        _right.setDirection(Servo.Direction.REVERSE);
        _left.scaleRange(min, max);
        _right.scaleRange(min, max);
    }

    public void flipBack() {
        _left.setPosition(0);
        _right.setPosition(0);
    }

    public void flipFront() {
        _left.setPosition(1);
        _right.setPosition(1);
    }

    public void flipTo(double position) {
        _left.setPosition(position+_offset);
        _right.setPosition(position);
    }

    public boolean isOut() {
        return (_left.getPosition()>=(0.99) &&
                _right.getPosition()>=(0.99));
    }

    public boolean isIn() {
        return (_left.getPosition()<=(0.01) &&
                _right.getPosition()<=(0.01));
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