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
    ColorSensor backBeacon;
    ColorSensor colorSensorWLA;
    ColorSensor frontBeacon;
    public double colorSensorAverageValues(ColorSensor colorsensor) throws InterruptedException
    {
        double average = (colorsensor.red() + colorsensor.blue() + colorsensor.green())/3.0;
        return average;
    }

//    public void pushRedBeacon() throws InterruptedException {
//        //power: .15
//        //distance: 25
//        //move forward and push the correct beacon
//        if (beaconValue(backBeacon) == 1) {
//            telemetry.addData("back");
//        } else if (beaconValue(frontBeacon) == 1) {
//            moveBackBeacon();
//        }
//    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        backBeacon = hardwareMap.colorSensor.get("cSWL");
        backBeacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        backBeacon.enableLed(false);
        telemetry.addData("colorSensorWL", "initialized");
        colorSensorWLA = hardwareMap.colorSensor.get("cSWA");
        colorSensorWLA.setI2cAddress(I2cAddr.create8bit(0x2a));
        colorSensorWLA.enableLed(true);
        telemetry.addData("colorSensorWLA", "initialized");
        frontBeacon = hardwareMap.colorSensor.get("cSB");
        frontBeacon.setI2cAddress(I2cAddr.create8bit(0x2e));
        frontBeacon.enableLed(false);
        telemetry.addData("colorSensorB", "initialized");

        telemetry.update();
        waitForStart();
        while(true)
        {
            telemetry.addData("cBeaconBlue", frontBeacon.blue());
            telemetry.update();
            sleep(1000);
            telemetry.addData("cBeaconRed", backBeacon.red());
            telemetry.update();
            sleep(1000);
            telemetry.addData("cWLA", colorSensorAverageValues(colorSensorWLA));
            telemetry.update();
            sleep(1000);
            idle();
        }
    }
}
