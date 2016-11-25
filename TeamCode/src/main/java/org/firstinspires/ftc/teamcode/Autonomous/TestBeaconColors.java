package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by sopa on 11/18/16.
 */
@Autonomous(name = "TestBeaconColors", group = "Autonomous")
public class TestBeaconColors extends LinearOpMode
{
    ColorSensor colorSensorBeacon;
    public double colorSensorAverageValues() throws InterruptedException
    {
        double average = (colorSensorBeacon.red() + colorSensorBeacon.blue() + colorSensorBeacon.green())/3.0;
        return average;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensorBeacon = hardwareMap.colorSensor.get("cSB");
        colorSensorBeacon.enableLed(true);
        telemetry.addData("colorSensor", "initialized");
        waitForStart();
        while(true)
        {
            telemetry.addData("cAverage", colorSensorAverageValues());
            telemetry.update();
            sleep(500);
            idle();
        }
    }
}
