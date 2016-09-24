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
public abstract class AutoOpMode extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    SensorAdafruitIMU IMU;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int BRV, BLV;
    int avg;

    public AutoOpMode()
    {
        super();
    }

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
        //initalize sensors
        IMU = new SensorAdafruitIMU();
        //gyro init
    }

    //Basic base movement
    public void moveForward(double power)
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }
    public void moveBackward(double power)
    {
        moveForward(-power);
    }
    public void turnRight(double power)
    {
        FR.setPower(-power);
        BR.setPower(-power);
        FL.setPower(-power);
        BL.setPower(-power);
    }
    public void turnLeft(double power)
    {
        turnRight(-power);
    }

    //Methods based soley on encoders
    public void getAvg() throws InterruptedException {
        sleep(500);
        BRV = Math.abs(BR.getCurrentPosition());
        BLV = Math.abs(BL.getCurrentPosition());
        avg = (BRV + BLV)/2;

    }
    public void moveForwardWithEncoders(double power, int distance)
    {


    }

}
