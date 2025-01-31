package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Claw;

public class IntakeClaw extends Claw {
    public static final double GRAB_POSITION = 1;
    public static final double RELEASE_POSITION = 0;

    public IntakeClaw(HardwareMap hardwareMap, String left, double min, double max) {
        super(hardwareMap, left, min, max);
    }

    public void grab() {
        openTo(GRAB_POSITION);
    }

    public void release() {
        openTo(RELEASE_POSITION);
    }
}
