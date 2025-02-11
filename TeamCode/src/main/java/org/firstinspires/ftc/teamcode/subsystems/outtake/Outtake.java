//package org.firstinspires.ftc.teamcode.subsystems.outtake;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//public class Outtake {
//
//    public boolean scoreHighBasket(Robot robot, ElapsedTime timer, double startTime) {
//        if(startTime == -1) return false;
//        if (timer.seconds() >= startTime+0 && timer.seconds() <= startTime+0.1) {
//            outtakeFlipOut();
//        }
//        if (timer.seconds() > 1.6) {
//            outtakeLetGo();
//        }
//        if (timer.seconds() > 1.8) {
//            outtakeFlipIn();
//        }
//        return true;
//    }
//
//}
