package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoSlide {
    private final Servo _left;
    private final Servo _right;

    public ServoSlide(HardwareMap hardwareMap, String left, String right, double min, double max) {
        _left = hardwareMap.get(Servo.class, left);
        _right = hardwareMap.get(Servo.class, right);
        _left.setDirection(Servo.Direction.FORWARD);
        _right.setDirection(Servo.Direction.REVERSE);
        _left.scaleRange(min, max);
        _right.scaleRange(min, max);
    }

    public void goTo(double position) {
        _left.setPosition(position);
        _right.setPosition(position);
    }

    public void retract(double increment) {
        double newLeftPosition = _left.getPosition() - increment;
        double newRightPosition = _right.getPosition() - increment;

        _left.setPosition(Math.max(newLeftPosition, 0));
        _right.setPosition(Math.max(newRightPosition, 0));
    }

    public void extendFull() {
        _left.setPosition(1);
        _right.setPosition(1);
    }

    public void retractFull(){
        _left.setPosition(0);
        _right.setPosition(0);
    }

    public boolean isFullExtended() {
        return (_left.getPosition() >= (0.99) &&
                _right.getPosition() >= (0.99));
    }

    public boolean isFullRetracted(){
        return (_left.getPosition() <= (0.01) &&
                _right.getPosition() <= (0.01));
    }

    public double getLeftPosition() {
        return _left.getPosition();
    }

    public double getRightPosition() {
        return _right.getPosition();
    }

}