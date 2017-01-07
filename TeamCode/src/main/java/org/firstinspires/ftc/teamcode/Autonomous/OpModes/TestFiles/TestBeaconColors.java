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
public class TestBeaconColors extends LinearOpMode
{
    ColorSensor colorSensorBeaconBlue;
    ColorSensor colorSensorBeaconRed;
    public double colorSensorRed(ColorSensor color) throws InterruptedException {
        return color.red();
    }
    public double colorSensorBlue(ColorSensor color) throws InterruptedException {
        return color.blue();
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensorBeaconBlue = hardwareMap.colorSensor.get("cSB");
        colorSensorBeaconRed = hardwareMap.colorSensor.get("cSR");
        colorSensorBeaconBlue.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorBeaconBlue.enableLed(false);
        colorSensorBeaconRed.setI2cAddress(I2cAddr.create8bit(0x2e));
        telemetry.addData("colorSensor", "initialized");
        telemetry.addData("test1", "initialized");
        waitForStart();
        telemetry.addData("cRed", colorSensorRed(colorSensorBeaconRed));
        telemetry.update();
        sleep(1500);
        telemetry.addData("cBlue", colorSensorBlue(colorSensorBeaconRed));
        telemetry.update();
        sleep(3000);
        idle();
    }
}
