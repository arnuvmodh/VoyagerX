package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

public class Intake {
    public boolean sampleFast(Robot robot, ElapsedTime timer, double startTime) {
        if(startTime==-1) return false;

        if (timer.seconds() >= startTime+0 && timer.seconds() <= startTime+1) {
            robot.clawPivot.flipTo(0.5);
            robot.horizontalSlide.goTo(Math.pow((timer.seconds()-startTime), 0.5)); // (2x)^0.5
            robot.spintake.spinIn(1);
        }
        if (timer.seconds() >= startTime+1 && timer.seconds() <= startTime+1.8) {
            robot.outtakePivot.flipFront();
            robot.intakePivot.flipIn();
            robot.horizontalSlide.retractFull();
        }
        if (timer.seconds() >= startTime+1.9 && timer.seconds() <= startTime+2) {
            robot.outtakeClaw.grab();
        }
        if (timer.seconds() >= startTime+2 && timer.seconds() <= startTime+2.2) {
            robot.horizontalSlide.goTo(0.3);
            robot.spintake.spinOut(1);
        }
        return timer.seconds() > startTime+2.2;
    }
}
