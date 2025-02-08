package org.firstinspires.ftc.teamcode.subsystems;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SampleDetector {
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private String color = "None";

    private float hsvValues[] = {0F, 0F, 0F};
    private final float values[] = hsvValues;
    private final double SCALE_FACTOR = 255;

    private int relativeLayoutId;
    private final View relativeLayout;

    public SampleDetector(HardwareMap hardwareMap, String sensorName) {
        sensorColor = hardwareMap.get(ColorSensor.class, sensorName);
        sensorDistance = hardwareMap.get(DistanceSensor.class, sensorName);
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    public String getColor() {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if(hsvValues[0]>=225&&hsvValues[0]<=228) {
                color = "Blue";
            }
            else if(hsvValues[0]>=17&&hsvValues[0]<=20) {
                color = "Red";
            }
            else if(hsvValues[0]>=70&&hsvValues[0]<=90) {
                color = "Yellow";
            }
            else {
                color = "None";
            }
            return color;
    }
}