package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sopa on 11/28/16.
 */
public abstract class AutoOpMode extends LinearOpMode

{
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    DcMotor ManIn;
    Servo Beacon;
    //average encoder value
    int beforeALV = 0;
    int beforeMLV = 0;
    double beforeAngle = 2;
    final double blackAVC = 2;
    final double whiteACV = 27;
    final double CORRECTION = .02;
    int FRV = 0;
    int FLV = 0;
    int avg = 0;
    long beforeTime = 0;
    long currentTime = 0;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    ColorSensor colorSensorWL;
    ColorSensor colorSensorBeacon;
    public void initalize() throws InterruptedException
    {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        Beacon = hardwareMap.servo.get("Beacon");
        Beacon.setPosition(.6);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("gyro", "initalizing");
        telemetry.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode0
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
        telemetry.addData("gyro", "initalized");
        telemetry.update();
        colorSensorWL = hardwareMap.colorSensor.get("cSWL");
        colorSensorWL.setI2cAddress(I2cAddr.create8bit(0x2a));
        telemetry.addData("colorSensorL", "initalized");
        telemetry.update();
        colorSensorBeacon = hardwareMap.colorSensor.get("cSB");
        colorSensorBeacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        telemetry.addData("colorSensorB", "initalized");
        telemetry.update();
        telemetry.addData("test1", "initalized");
        telemetry.update();

    }
    //movement methods
    public void zero() throws InterruptedException
    {
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveForward(double power) throws InterruptedException
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }

    public void moveBackward(double power) throws InterruptedException
    {
        moveForward(-power);
    }

    public void turnRight(double power) throws InterruptedException
    {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        BL.setPower(power);
    }

    public void turnLeft(double power) throws InterruptedException
    {
        turnRight(-power);
    }

    //Shooter
    public void bringDownShooter(double power, int distance) throws InterruptedException
    {
        int beforePos = Math.abs(ManLift.getCurrentPosition());
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
        while (Math.abs(ManLift.getCurrentPosition() - beforePos) < distance)
        {
            ManLift.setPower(power);
            idle();
        }
        ManLift.setPower(0);
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
    }
    //might want to add two servos so we can shoot faster. Ask Sachin
    public void shoot(double power, int distance) throws InterruptedException
    {
        int beforeManLift = ManLift.getCurrentPosition();
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        bringDownShooter((.21 * -1), (distance + (Math.abs(beforeManLift - 1100))));
        sleep(1000);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 3000)
            idle();
        ManIn.setPower(-.15);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 2500)
            idle();
        ManIn.setPower(0);
        ShooterF.setPower(0);
        ShooterB.setPower(0);
        ManIn.setPower(0);
    }

    //Sensor methods

    //color sensor
    public double colorSensorAverageValues(ColorSensor sensor) throws InterruptedException
    {
        double average = (sensor.red() + sensor.blue() + sensor.green())/3.0;
        return average;
    }
    public double colorSensorRed(ColorSensor sensor) throws InterruptedException
    {
        return colorSensorBeacon.red();
    }
    public double colorSensorBlue(ColorSensor sensor) throws InterruptedException
    {
        return colorSensorBeacon.blue();
    }

    //gyro methods
    public float getGyroYaw() throws InterruptedException
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    //encoders
    public int getAvg() throws InterruptedException
    {
        FRV = Math.abs(FR.getCurrentPosition());
        FLV = Math.abs(FL.getCurrentPosition());
        avg = Math.abs((FRV + FLV)/2);
        return avg;
    }

    //Forwards, Backwards and Turning

    //forward
    public void moveForwardWithEncoders(double power, int distance) throws InterruptedException
    {
        telemetry.addData("encodersR", getAvg());
        telemetry.update();
        beforeALV = getAvg();
        //original moveForward

        while(Math.abs(getAvg() - beforeALV) < distance)
        {
            moveForward(power);
            idle();
        }
        telemetry.addData("encodersR", getAvg());
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveBackWardWithEncoders(double power, int distance) throws InterruptedException
    {
        moveForwardWithEncoders(-power, distance);
    }

    //turn right
    public void turnRightWithGyro(double power, double angle) throws InterruptedException
    {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle)
        {
            turnRight(power);
            idle();
        }
        beforeAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
    }

    //turn left
    public void turnLeftWithGyro(double power, double angle) throws InterruptedException
    {
        turnRightWithGyro(-power, angle);
    }

    public void turnRightWithPID(double power, double angle) throws InterruptedException
    {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle)
        {
            turnRight(power);
            idle();
        }
        beforeAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
    }

    //gyro stabilization
    public void moveForwardWithCorrection(double power, int distance) throws InterruptedException {
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = CORRECTION;
        while (Math.abs(getAvg() - beforeALV) < distance) {
            FR.setPower(power);
            BR.setPower(power);
            FL.setPower(-power);
            BL.setPower(-power);
            double difference = Math.abs(getGyroYaw() - beforeAngle);
            while (difference > 2)
            {
                FR.setPower(power * (1 + difference * correction));
                BR.setPower(power * (1 + difference * correction));
                FL.setPower(-power);
                BL.setPower(-power);
                telemetry.addData("LeftPower", FR.getPower());
                telemetry.addData("RightPower", BR.getPower());
                telemetry.update();
                difference = Math.abs(getGyroYaw() - beforeAngle);
                idle();
            }
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        telemetry.update();
        if (Math.abs(beforeAngle - getGyroYaw()) < 2) {
            telemetry.addData("success", "correction works");
            telemetry.update();
        }
        else
        {
            telemetry.addData("success", "correction failed");
            telemetry.update();
        }
    }

    public void moveBackWardWithCorrection(double power, int distance) throws InterruptedException
    {
        moveForwardWithCorrection(-power, distance);
        idle();
    }

    //beacon pushing methods
    public void moveToWhiteLine(double power, int distance) throws InterruptedException
    {
        beforeALV = getAvg();
        //barely move forward until white line is sensed
        while (Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10)
        {
            moveForwardWithCorrection(.15, 400);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("encodersA", getAvg());
        telemetry.addData("colorAverage", colorSensorAverageValues(colorSensorWL));
        telemetry.update();
        sleep(1000);
    }

    public void pushRedBeacon(double power, int distance) throws InterruptedException
    {
        //power: .15
        //distance: 25

        //move forward and push the correct beacon
        if (colorSensorRed(colorSensorBeacon) < colorSensorBlue(colorSensorBeacon)) {
            beforeALV = getAvg();
            moveBackWardWithCorrection(power, distance);
            Beacon.setPosition(.43);
            beforeALV = getAvg();
            moveForwardWithCorrection(power, distance);
            telemetry.addData("hit1", "rip");
            sleep(3000); //change sleep values when this part works
            Beacon.setPosition(1);
            beforeALV = getAvg();
            moveForwardWithCorrection(power, distance);
            idle();
        }
        else
        {
            Beacon.setPosition(.43);
            sleep(2000);
            moveBackWardWithCorrection(.15, 40);
            moveForwardWithCorrection(.15, 40);
            idle();
            telemetry.addData("hit2", "rip");
            Beacon.setPosition(1);
            telemetry.addData("encodersA", getAvg());
            beforeALV = getAvg();
        }
        sleep(2000);
    }

    public void pushBlueBeacon(double power, int distance) throws InterruptedException
    {
        //power: .15
        //distance: 25

        //move forward and push the correct beacon
        if (colorSensorRed(colorSensorBeacon) > colorSensorBlue(colorSensorBeacon)) {
            beforeALV = getAvg();
            moveBackWardWithCorrection(power, distance);
            Beacon.setPosition(.43);
            beforeALV = getAvg();
            moveForwardWithCorrection(power, distance);
            telemetry.addData("hit1", "rip");
            sleep(3000); //change sleep values when this part works
            Beacon.setPosition(1);
            beforeALV = getAvg();
            moveForwardWithCorrection(power, distance);
            idle();
        }
        else
        {
            Beacon.setPosition(.43);
            sleep(2000);
            moveBackWardWithCorrection(.15, 40);
            moveForwardWithCorrection(.15, 40);
            idle();
            telemetry.addData("hit2", "rip");
            Beacon.setPosition(1);
            telemetry.addData("encodersA", getAvg());
            beforeALV = getAvg();
        }
        sleep(2000);
    }

}
