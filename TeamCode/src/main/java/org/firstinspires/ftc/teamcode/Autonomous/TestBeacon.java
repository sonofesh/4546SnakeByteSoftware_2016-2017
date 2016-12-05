package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by sopa on 11/18/16.
 */
@Autonomous(name = "TestBeacon", group = "Autonomous")
@Disabled
public class TestBeacon extends LinearOpMode
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
        colorSensorBeacon = hardwareMap.colorSensor.get("cSWL");
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
