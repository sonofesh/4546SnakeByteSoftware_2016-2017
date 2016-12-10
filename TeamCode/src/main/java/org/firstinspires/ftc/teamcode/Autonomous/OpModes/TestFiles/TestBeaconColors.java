package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by sopa on 11/18/16.
 */
@Autonomous(name = "TestBeaconColors", group = "Autonomous")
@Disabled
public class TestBeaconColors extends LinearOpMode
{
    ColorSensor colorSensorBeacon;
    public double colorSensorRed() throws InterruptedException
    {
        return colorSensorBeacon.red();
    }
    public double colorSensorBlue() throws InterruptedException
    {
        return colorSensorBeacon.blue();
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensorBeacon = hardwareMap.colorSensor.get("cSB");
        colorSensorBeacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorBeacon.enableLed(true);
        telemetry.addData("colorSensor", "initialized");
        telemetry.addData("test6", "initialized");
        waitForStart();
            telemetry.addData("cRed", colorSensorRed());
            telemetry.update();
            sleep(1500);
            telemetry.addData("cBlue", colorSensorBlue());
            telemetry.update();
            sleep(1500);
            idle();
    }
}
