package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sopa on 11/19/16.
 */
@Autonomous(name = "GyroTest", group = "Autonomous")
public class GyroTest extends LinearOpMode
{

    public BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    public float getGryoYaw() throws InterruptedException
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }
    public float getGryoPitch() throws InterruptedException
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.secondAngle;
    }
    public float getGryoRoll() throws InterruptedException
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.thirdAngle;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("gyro", "initalizing");
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
        telemetry.addData("gyro", "initalized");
        telemetry.update();
        waitForStart();
        sleep(5000);
        telemetry.addData("yawAngle", getGryoYaw());
        telemetry.update();
        sleep(2000);
        telemetry.addData("pitchAngle", getGryoPitch());
        telemetry.update();
        sleep(2000);
        telemetry.addData("rollAngle", getGryoRoll());
        telemetry.update();
        sleep(2000);
    }
}
