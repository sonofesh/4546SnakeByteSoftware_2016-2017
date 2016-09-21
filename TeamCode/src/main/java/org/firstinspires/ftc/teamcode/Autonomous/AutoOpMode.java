package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitIMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    BNO055IMU IMU;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int BRV, FRV;
    int avg;

    public void initialize(String side) throws InterruptedException
    {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        telemetry.addData("gyro", "initializing");


    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        


    }
    public void moveForwardWithEncoders(){

    }
}
