package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitIMU;

/**
 * Created by sopa on 9/19/16.
 */
@TeleOp(name = "AutoOpMode", group = "Autonomous")
@Disabled
public class AutoOpMode extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    SensorAdafruitIMU gyroSensor;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int BRV, FRV;
    int avg;


    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}
