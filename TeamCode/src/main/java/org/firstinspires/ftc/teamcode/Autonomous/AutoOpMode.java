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
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitIMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Created by 4546SnakeByte on 9/19/16.
 */
@Autonomous(name = "AutoOpMode", group = "Autonomous")
@Disabled
public abstract class AutoOpMode extends LinearOpMode
{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    ColorSensor colorSensorWL;
    ColorSensor colorSensorWL2;
    ColorSensor colorSensorBeacon;
    //values for whiteline
    public static int redValue = 0;
    public static int greenValue = 0;
    public static int blueValue = 0;
    public static int alphaValue = 0;
    //values for red beacon
    public static int redValueR = 0;
    public static int greenValueR = 0;
    public static int blueValueR = 0;
    public static int alphaValueR = 0;
    //values for blue beacon
    public static int redValueB = 0;
    public static int greenValueB = 0;
    public static int blueValueB = 0;
    public static int alphaValueB = 0;
    public int[][] colors;
    int BRV, BLV;
    int avg;
    Orientation angles;
    Acceleration gravity;
    SensorAdafruitIMU imu;
    int standardBRV = 0;
    int standardBLV = 0;
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
        telemetry.addData("motors", "initialized");
        telemetry.update();
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        //Initialize Sensors
        telemetry.addData("gyro", "initializing");
        telemetry.update();
        //set up parameters for gyro sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = new SensorAdafruitIMU();
        //Color Sensor Initialization
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        colorSensorWL2 = hardwareMap.colorSensor.get("cSWL2");
        colorSensorBeacon = hardwareMap.colorSensor.get("cSBeacon");
        colorSensorWL.enableLed(true);
        colorSensorWL2.enableLed(true);
        colorSensorBeacon.enableLed(true);
        colors = new int[3][4];
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
    public void reset() throws InterruptedException
    {
        standardBRV = Math.abs(BR.getCurrentPosition()) - standardBRV;
        standardBLV = Math.abs(BL.getCurrentPosition()) - standardBLV;
        BRV -= standardBRV;
        BLV -= standardBLV;
        imu
    }
    //Methods based solely on encoders
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
    //Gyro methods
    //Movement methods that correct with gyros
    public void moveForwardWithEncodersCorrectingWithGyros(double power, int distance) throws InterruptedException
    {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        while (getAvg() < distance)
        {
            moveForward(power);
            if (imu.getYaw)
        }
        reset();
    }
    public void turnRightWithGyro()
    {

    }

    public void composeGyroTelemetry()
    {
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)));
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle)));
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle)));
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }
    //Color sensor methods
    public void getColors(ColorSensor input)
    {
    }
    public boolean isOnWhiteLine()
    {


        return false;
    }
}
