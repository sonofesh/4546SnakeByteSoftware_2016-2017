package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
    DcMotor ManIn;
    DcMotor ManLift;
    ColorSensor colorSensorWL;
    ColorSensor colorSensorBeacon;
    ColorSensor colorSensorBeaconL;
    //values for whiteline
    public static int redValue = 0;
    public static int greenValue = 0;
    public static int blueValue = 0;
    public static int alphaValue = 0; //not sure if this will be used
    //values for red beacon
    public static int redValueR = 0;
    public static int greenValueR = 0;
    public static int blueValueR = 0;
    public static int alphaValueR = 0; //not sure if this will be used
    //values for blue beacon
    public static int redValueB = 0;
    public static int greenValueB = 0;
    public static int blueValueB = 0;
    public static int alphaValueB = 0; //not sure if this will be used
    public int[][] colors;
    int BRV, BLV;
    int avg;
    //volatile double[] anglesM = new double[2];
    public static BNO055IMU imu;
    public static BNO055IMU.Parameters parameters;
    ModernRoboticsI2cRangeSensor rangeSensor;
    int standardBRV = 0;
    int standardBLV = 0;
    int wallDistance = 20; //adjust distance later
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
        DcMotor ManIn = hardwareMap.dcMotor.get("ManIn");
        DcMotor ManLift = hardwareMap.dcMotor.get("ManLift");
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
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("gyro", "initialized");
        //Color Sensor Initialization
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        //colorSensorBeaconR = hardwareMap.colorSensor.get("cSWL2");
        colorSensorBeacon = hardwareMap.colorSensor.get("cSBeacon");
        colorSensorWL.setI2cAddress(I2cAddr.create7bit(0x20));
        //colorSensorBeaconR.setI2cAddress(I2cAddr.create7bit(0x20));
        colorSensorBeacon.setI2cAddress(I2cAddr.create7bit(0x20));
        colorSensorWL.enableLed(true);
        //colorSensorBeaconR.enableLed(true);
        //colorSensorBeaconL.enableLed(true);
        //colors = new int[3][4];
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
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
    public float getGryoYaw()
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }
    public float getGyroPitch()
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.secondAngle;
    }
    public float getGyroRoll()
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.thirdAngle;
    }
    //Movement methods that correct with gyros
    public void moveForwardWithEncodersCorrectingWithGyros(double power, int distance) throws InterruptedException
    {
        float beforeA = getGryoYaw();
        float afterA = 0;
        float error = 0;
        while (getAvg() < distance)
        {
            beforeA = getGryoYaw();
            error = afterA - beforeA;
            moveForward(power);
            if (Math.abs(error) > 2)
            {
                if(error > 0)
                    turnRightWithGyro(.1, error);
                else
                    turnLeftWithGyro(.1, error * -1);
            }
            afterA = getGryoYaw();
        }
        reset();
    }
    public void turnRightWithGyro(double power, float angle)
    {
        float beforeA = getGryoYaw();
        float afterA = 0;
        float error = 0;
        while(angle < getGryoYaw())
        {
            beforeA = getGryoYaw();
            error = afterA - beforeA;
            turnRight(power);
            if (Math.abs(error) > 2)
            {
                if(error > 0)
                    turnRightWithGyro(.1, error);
                else
                    turnLeftWithGyro(.1, error * -1);
            }
            afterA = getGryoYaw();
        }
    }

    public void turnLeftWithGyro(double power, float angle)
    {
        turnRightWithGyro(-power, angle);
    }
    //Color sensor methods
    public int[] getColors(ColorSensor input)
    {
        int[] colorArray = {input.red(), input.green(), input.blue()};
        return colorArray;
    }
    public boolean isOnWhiteLine(ColorSensor input)
    {
        if(Math.abs(redValue - input.red()) < 50)
        {
            if((Math.abs(blueValue - input.blue()) < 50))
            {
                if((Math.abs(greenValue - input.green()) < 50))
                {
                    return true;
                }
            }
        }
        return false;
    }
    public boolean isBlue(ColorSensor input)
    {
        if(Math.abs(input.red()) > 100)
            return true;
        return false;
    }
    //Whiteline and gyro methods
    public void stopAtWhiteLine(double power) throws InterruptedException {
        while(!isOnWhiteLine(colorSensorWL))
            moveForwardWithEncodersCorrectingWithGyros(.1, 1000);
    }
    public double getDistance()
    {
        return rangeSensor.cmOptical();
    }
    /**public void approachWall(int power, int distance, int spaceFromWall)
    {
        while()
        {
            moveForwardWithEncoders();
        }
    }*/
    public void composeTelemetry()
    {
        final Orientation[] angles = {imu.getAngularOrientation()};
        final Acceleration[] gravity = {imu.getGravity()};
        telemetry.addAction(new Runnable()
        {
            @Override
            public void run()
            {
                angles[0] = imu.getAngularOrientation();
                gravity[0] = imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles[0].angleUnit, angles[0].firstAngle)));
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles[0].angleUnit, angles[0].secondAngle)));
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles[0].angleUnit, angles[0].thirdAngle)));
                    }
                });

        telemetry.addLine().addData("grvty", new Func<String>() {
            @Override
            public String value() {
                return gravity[0].toString();
            }
        }).addData("mag", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity[0].xAccel * gravity[0].xAccel + gravity[0].yAccel * gravity[0].yAccel + gravity[0].zAccel * gravity[0].zAccel));
            }
        });
        telemetry.addLine();
            telemetry.addData("Red  ", colorSensorWL.red());
            telemetry.addData("Green", colorSensorWL.green());
            telemetry.addData("Blue ", colorSensorWL.blue());
        telemetry.addLine();
            telemetry.addData("WhiteLine?", isOnWhiteLine(colorSensorWL));
    }
}
