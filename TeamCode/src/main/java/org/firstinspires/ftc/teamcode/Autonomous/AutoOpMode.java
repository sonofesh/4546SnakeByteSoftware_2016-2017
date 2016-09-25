package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "AutoOpMode", group = "Autonomous")
@Disabled
public abstract class AutoOpMode extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    BNO055IMU imu;;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int BRV, BLV;
    int avg;

    public AutoOpMode()
    {
        super();
    }

    public void initialize() throws InterruptedException
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
        //Initialize Sensors
        //Color Sensor Initialization
        //set up parameters for gyro sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Color Sensor Initialization

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRV = 0;
        BLV = 0;
        reset();
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
    public void stopBase()
    {
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }
    //Methods based solely on encoders
    public void reset() throws InterruptedException
    {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRV = 0;
        BLV = 0;
        stop();
    }
    public int getAvg() throws InterruptedException
    {
        BRV = Math.abs(BR.getCurrentPosition());
        BLV = Math.abs(BL.getCurrentPosition());
        avg = (BRV + BLV)/2;
        return avg;
    }
    public void moveForwardWithEncoders(double power, int distance) throws InterruptedException
    {
        while(getAvg() < distance)
        {
            moveForward(power);
        }
        reset();
    }
    public void moveBackWardWithEncoders(double power, int distance) throws InterruptedException
    {
        while(getAvg() < distance)
        {
            moveBackward(power);
        }
        reset();
    }
    //Gyro based turning methods
    public void turnRightWithGyro()
    {

    }

}
