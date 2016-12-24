package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sopa on 11/29/16.
 */
@Autonomous(name = "SetRobot", group = "Autonomous")
public class setRobot extends LinearOpMode
{
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    double beforeAngle;
    //gyro methods
    public float getGryoYaw() throws InterruptedException
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("gyro", "initializing");
        telemetry.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
        telemetry.addData("gyro", "initialized");
        telemetry.update();
        beforeAngle = getGryoYaw();
//        while(Math.abs(getGryoYaw()-beforeAngle) != 15)
        while(true)
        {
//            telemetry.addData("angle", (Math.abs(getGryoYaw()-beforeAngle)));
//            telemetry.update();
            telemetry.addData("angle", getGryoYaw());
            telemetry.update();
            idle();
        }
    }
}
