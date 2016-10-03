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
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

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
    BNO055IMU imu;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    int BRV, BLV;
    int avg;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;

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
        telemetry.addData("gyro", "initializing");
        telemetry.update();
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
    //
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
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRV = 0;
        BLV = 0;
        imu.initialize(parameters);
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
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        gravity  = imu.getGravity();
        while(getAvg() < distance)
        {
            moveForward(power);
        }
        reset();
    }
    public void moveBackWardWithEncoders(double power, int distance) throws InterruptedException
    {
        while(getAvg() < distance) {
            moveBackward(power);
        }
        reset();
    }
    //Gyro methods
    //Movement methods that correct with gyros
    public void moveForwardWithEncodersCorrectingWithGyros(double power, int distance) throws InterruptedException
    {

        while (getAvg() < distance)
        {

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
}
