package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
/**
 * Created by sopa on 11/18/16.
 */
@Autonomous(name = "TestWhiteLine", group = "Autonomous")
public class TestColor extends LinearOpMode
{
    ColorSensor colorSensorWL;
    public double colorSensorAverageValues() throws InterruptedException
    {
        double average = (colorSensorWL.red() + colorSensorWL.blue() + colorSensorWL.green())/3.0;
        return average;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        colorSensorWL.setI2cAddress(I2cAddr.create8bit(0x2a));
        colorSensorWL.enableLed(true);
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
