package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private final DcMotor _frontLeftMotor;
    private final DcMotor _backLeftMotor;
    private final DcMotor _frontRightMotor;
    private final DcMotor _backRightMotor;
    public Drive(HardwareMap hardwareMap, String frontLeft, String backLeft, String frontRight, String backRight) {
        _frontLeftMotor = hardwareMap.dcMotor.get(frontLeft);
        _backLeftMotor = hardwareMap.dcMotor.get(backLeft);
        _frontRightMotor = hardwareMap.dcMotor.get(frontRight);
        _backRightMotor = hardwareMap.dcMotor.get(backRight);

        _frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        _backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        _frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        _backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        _frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
