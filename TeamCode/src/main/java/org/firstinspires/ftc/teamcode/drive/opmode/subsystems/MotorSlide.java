package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSlide {
    private final DcMotor _left;
    private final DcMotor _right;
    private final int _min;
    private final int _max;

    public MotorSlide(HardwareMap hardwareMap, String left, String right, int min, int max) {
        _left = hardwareMap.get(DcMotor.class, left);
        _right = hardwareMap.get(DcMotor.class, right);
        _min = min;
        _max = max;
        _left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _left.setDirection(DcMotorSimple.Direction.REVERSE);
        _right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void goTo(int targetPosition, double power) {
        _left.setTargetPosition(targetPosition);
        _right.setTargetPosition(targetPosition);

        _left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _left.setPower(Math.abs(power));
        _right.setPower(Math.abs(power));

        if(_left.isBusy() || _right.isBusy()) return;

        _left.setPower(0);
        _right.setPower(0);

        _left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset() {
        _left.setPower(0);
        _right.setPower(0);

        _left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extend(float power) {
        if (_left.getCurrentPosition() < _max-50) {
            _left.setPower(power);
        } else {
            _left.setPower(0);
        }

        if (_right.getCurrentPosition() < _max-50) {
            _right.setPower(power);
        } else {
            _right.setPower(0);
        }
    }

    public void retract(float power) {
        if (_left.getCurrentPosition() > _min+50) {
            _left.setPower(-power);
        } else {
            _left.setPower(0);
        }

        if (_right.getCurrentPosition() > _min+50) {
            _right.setPower(-power);
        } else {
            _right.setPower(0);
        }
    }

    public void extendFull() {
        if (_left.getCurrentPosition() < _max-50) {
            _left.setPower(1);
        } else {
            _left.setPower(0);
        }

        if (_right.getCurrentPosition() < _max-50) {
            _right.setPower(1);
        } else {
            _right.setPower(0);
        }
    }

    public void retractFull() {
        if (_left.getCurrentPosition() > _min+50) {
            _left.setPower(-1);
        } else {
            _left.setPower(0);
        }

        if (_right.getCurrentPosition() > _min+50) {
            _right.setPower(-1);
        } else {
            _right.setPower(0);
        }
    }

    public void stop() {
        _left.setPower(0);
        _right.setPower(0);
    }

    public boolean isFullyExtended() {
        return (_left.getCurrentPosition() >= (_max-100) && _right.getCurrentPosition() >= (_max-100));
    }

    public boolean isFullyRetracted() {
        return (_left.getCurrentPosition() <= (_min+100) && _right.getCurrentPosition() <= (_min+100));
    }

    public boolean isAt(double position) {
        return (_left.getCurrentPosition() <= (position+100) && _left.getCurrentPosition() >= (_min-100) &&
                _right.getCurrentPosition() <= (position+100) && _right.getCurrentPosition() >= (_min-100));
    }

    public int getLeftPosition() {
        return _left.getCurrentPosition();
    }

    public int getRightPosition() {
        return _right.getCurrentPosition();
    }
}