package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
/**
 * Created by sopa on 11/18/16.
 */
@Autonomous(name = "TestColor", group = "Autonomous")
public class TestColor extends LinearOpMode
{
    ColorSensor colorSensorWL;
    ColorSensor colorSensorWLA;
    ColorSensor colorSensorBeacon;
    public double colorSensorAverageValues(ColorSensor colorsensor) throws InterruptedException
    {
        double average = (colorsensor.red() + colorsensor.blue() + colorsensor.green())/3.0;
        return average;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        colorSensorWL.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorWL.enableLed(true);
        telemetry.addData("colorSensorWL", "initialized");
        colorSensorWLA = hardwareMap.colorSensor.get("cSWA");
        colorSensorWLA.setI2cAddress(I2cAddr.create8bit(0x2a));
        colorSensorWLA.enableLed(true);
//        telemetry.addData("colorSensorWLA", "initialized");
        colorSensorBeacon = hardwareMap.colorSensor.get("cSB");
        colorSensorBeacon.setI2cAddress(I2cAddr.create8bit(0x2e));
        telemetry.addData("colorSensorB", "initialized");

        telemetry.update();
        waitForStart();
        while(true)
        {
//            telemetry.addData("cWL", colorSensorAverageValues(colorSensorWL));
//            telemetry.update();
//            sleep(1000);
            telemetry.addData("cBeaconBlue", colorSensorBeacon.blue());
            telemetry.update();
            sleep(1000);
            telemetry.addData("cBeaconRed", colorSensorBeacon.red());
            telemetry.update();
            sleep(1000);
            telemetry.addData("cWLA", colorSensorAverageValues(colorSensorWLA));
            telemetry.update();
            sleep(1000);
            idle();
        }
    }
}
