package org.firstinspires.ftc.teamcode.Autonomous.OpModes;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by sopa on 11/28/16.
 * Information: turnRight is negative whereas turnLeft is positive
 */

public abstract class AutoOpMode extends LinearOpMode {
    DcMotor FR;
    DcMotor BR;
    DcMotor FL;
    DcMotor BL;
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    Servo ManBeaconL;
    Servo ManBeaconR;
    DcMotor ManIn;
    Servo BackBeaconPusher;
    Servo FrontBeaconPusher;
    //average encoder value
    int beforeALV = 0;
    double beforeAngle = 2;
    final double whiteACV = 27;
    final double CORRECTION = .13;
    int FRV = 0;
    int FLV = 0;
    int avg = 0;
    long beforeTime = 0;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    ColorSensor frontBeacon;
    ColorSensor colorSensorWLA;
    //ColorSensor colorSensorBlueBeacon;
    ColorSensor backBeacon;
    ModernRoboticsI2cRangeSensor wallSensor;
    int count = 0;
    public void initialize() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        BackBeaconPusher = hardwareMap.servo.get("BeaconL");
        FrontBeaconPusher = hardwareMap.servo.get("BeaconR");

//        ManBeaconL = hardwareMap.servo.get("ManBeaconL");
//        ManBeaconR = hardwareMap.servo.get("ManBeaconR");
//        ManBeaconL.setPosition(.3);
//        ManBeaconR.setPosition(.7);
//        BlueBeaconPusher.setPosition(0);
//        RedBeaconPusher.setPosition(0);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontBeaconPusher.setPosition(.1);
        BackBeaconPusher.setPosition(.1);
        telemetry.addData("gyro", "initializing");
        telemetry.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
        telemetry.addData("gyro", "initialized");
        backBeacon = hardwareMap.colorSensor.get("cSWL");
        backBeacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        backBeacon.enableLed(false);
        telemetry.addData("colorSensorWL", "initialized");
        colorSensorWLA = hardwareMap.colorSensor.get("cSWA");
        colorSensorWLA.setI2cAddress(I2cAddr.create8bit(0x2a));
        colorSensorWLA.enableLed(true);
        telemetry.addData("colorSensorWLA", "initialized");
        frontBeacon = hardwareMap.colorSensor.get("cSB");
        frontBeacon.setI2cAddress(I2cAddr.create8bit(0x2e));
        frontBeacon.enableLed(false);
        telemetry.addData("colorSensorB", "initialized");
        wallSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wallSensor");
        telemetry.addData("range", getRawDistance());
        telemetry.update();
        telemetry.addData("backbeacon", backBeacon.red());
        telemetry.addData("colorSensorWLA", colorSensorWLA.red());
        telemetry.update();
    }
    //movement methods
    public void zero() throws InterruptedException {
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveForward(double power) throws InterruptedException {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
    }

    public void moveBackward(double power) throws InterruptedException {
        moveForward(-power);
    }

    public void turnRight(double power) throws InterruptedException {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        BL.setPower(power);
    }

    public void turnLeft(double power) throws InterruptedException {
        turnRight(-power);
    }

    //Shooter
    public void bringDownShooter(double power, int distance) throws InterruptedException {
        int beforePos = Math.abs(ManLift.getCurrentPosition());
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
        while (Math.abs(ManLift.getCurrentPosition() - beforePos) < distance) {
            ManLift.setPower(power);
            idle();
        }
        ManLift.setPower(0);
        telemetry.addData("ManLift", ManLift.getCurrentPosition());
        telemetry.update();
    }

    /**
     * The entire shooting process can be hastened with the addition of two servos that
     * would reduce the overall bringDownShooter() distance
     * @param power
     * @param distance
     * @throws InterruptedException
     */
    public void shoot(double power, int distance) throws InterruptedException {
        int beforeManLift = ManLift.getCurrentPosition();
        ShooterF.setPower(-power);
        ShooterB.setPower(power);
        bringDownShooter((.55 * -1), (distance + (Math.abs(beforeManLift - 1150))));
        sleep(1000);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 500)
            idle();
        ManIn.setPower(-.2);
        ShooterF.setPower(-power + .1);
        ShooterB.setPower(power - .1);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 1500)
            idle();
        ManIn.setPower(0);
        ShooterF.setPower(0);
        ShooterB.setPower(0);
        ManIn.setPower(0);
    }

    public void shootSlow(double power, int distance) throws InterruptedException
    {
        int beforeManLift = ManLift.getCurrentPosition();
        if(power < .8) {
            ShooterF.setPower(power * 1.25);
            ShooterB.setPower(-power * 1.25);
        }
        else {
            ShooterF.setPower(power);
            ShooterB.setPower(-power);
        }

        bringDownShooter((.2 * -1), (distance + (Math.abs(beforeManLift - 1100))));
        sleep(1000);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 3000)
            idle();
        ManIn.setPower(-.125);
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 500)
            idle();
        beforeTime = System.currentTimeMillis();
        while(Math.abs(System.currentTimeMillis() - beforeTime) < 2000)
            idle();
        ManIn.setPower(0);
        ShooterF.setPower(0);
        ShooterB.setPower(0);
        ManIn.setPower(0);
    }

    //Sensor methods

    //color sensor
    public double colorSensorAverageValues(ColorSensor sensor) throws InterruptedException {
        double average = (sensor.red() + sensor.blue() + sensor.green())/3.0;
        return average;
    }
    public double colorSensorAverageValues() throws InterruptedException {
        double average = (colorSensorWLA.red() + colorSensorWLA.blue() + colorSensorWLA.green())/3.0;
        return average;
    }
    public double colorSensorRed(ColorSensor sensor) throws InterruptedException {
        return sensor.red();
    }
    public double colorSensorBlue(ColorSensor sensor) throws InterruptedException {
        return sensor.blue();
    }

    //gyro methods
    public double getGyroYaw() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation();
        return (angles.firstAngle * -1);
    }

    public double getGyroYaw(double turn) throws InterruptedException {
        double turnAbs = Math.abs(turn);
        Orientation angles = imu.getAngularOrientation();
        if (turnAbs > 270 && Math.abs(angles.firstAngle) < 90)
            return (Math.abs(angles.firstAngle) - (turnAbs - 360));
        else if (turnAbs < 90 && Math.abs(angles.firstAngle) > 270)
            return ((Math.abs(angles.firstAngle) - 360) - turnAbs);
        return (Math.abs(angles.firstAngle) - turnAbs);
    }

    public double getVelocity() throws InterruptedException {
        return 0.0;
    }


    //encoders
    public int getAvg() throws InterruptedException {
        FRV = Math.abs(FR.getCurrentPosition());
        FLV = Math.abs(FL.getCurrentPosition());
        avg = Math.abs((FRV + FLV)/2);
        return avg;
    }

    //range sensor
//    public double getDist(ModernRoboticsI2cRangeSensor wallSensor) throws InterruptedException {
//        double value = range.getDistance(DistanceUnit.CM);
//        if(value == 255 && value == 0) {
//            sleep(62);
//            getDist(range);
//        }
//        return value;
//    }

    //voltage adjustment

    //base
    public double basePowerMultiplier()
    {
        double power = 1;
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage();
        voltage /= 2;
        if (voltage <= 13.5)
            power = 1.25;
        else if (voltage <= 13.75 && voltage > 13.5)
            power = 1;
        else if (voltage > 13.75 && voltage <= 13.9)
            power = .95;
        else if (voltage > 13.9 && voltage <= 14)
            power = .9;
        else if (voltage > 14)
            power = .85;
        return power;
    }

    //Forwards, Backwards and Turning

    //forward
    public void moveForwardWithEncoders(double power, int distance) throws InterruptedException {
        telemetry.addData("encodersR", getAvg());
        telemetry.update();
        beforeALV = getAvg();
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage())/2;
        while(Math.abs(getAvg() - beforeALV) < distance) {
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

    public void mashBeacons(double power, int distance) throws InterruptedException {
        telemetry.addData("encodersR", getAvg());
        telemetry.update();
        beforeALV = getAvg();
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (14 - voltageAverage) * 200;
        double firstTime = System.currentTimeMillis();
        while(Math.abs(getAvg() - beforeALV) < (distance + change) && System.currentTimeMillis() - firstTime < 1500) {
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


    public void moveBackWardWithEncoders(double power, int distance) throws InterruptedException {
        moveForwardWithEncoders(-power, distance);
    }

    //turn right
    public void turnRightWithGyro(double power, double angle) throws InterruptedException {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            turnRight(power);
            idle();
        }
        beforeAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnRightWithGyroOneSide(double power, double angle) throws InterruptedException {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            FR.setPower(power);
            BR.setPower(power);
            idle();
        }
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    //turn left
    public void turnLeftWithGyro(double power, double angle) throws InterruptedException {
        turnRightWithGyro(-power, angle);
    }

    public void turnRightWithPID(double angle) throws InterruptedException
    {
        //calibration constants
        double p = .004; double i = .000015; double d = 2.0;
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long lastTime = System.currentTimeMillis();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            error = angle - Math.abs(getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime/(error-pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if(output < .15)
                output = 0;
            //+ (reset * i) + derivative
            turnRight(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        if(Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.addData("turn", "success");
        else
            telemetry.addData("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnRightWithPID(double angle, double p, double i, double d) throws InterruptedException {
        //calibration constants
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        double change = 0;
//        if(Math.abs(360 - getGyroYaw()) < angle)
//            change = -1 * Math.abs(360 - angle);
//        else if(Math.abs(360 - getGyroYaw()) < angle)
//            change = Math.abs(360 - angle);
        beforeAngle = getGyroYaw() + change;
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long firstTime = System.currentTimeMillis();
        long lastTime = System.currentTimeMillis();
        while(Math.abs(change + getGyroYaw() - beforeAngle) < angle && System.currentTimeMillis() - firstTime < 3000) {
            error = angle - Math.abs(change + getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime/(error-pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if(output < .15)
                output = 0;
            //+ (reset * i) + derivative
            turnRight(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = change + getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        if(Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.addData("turn", "success");
        else
            telemetry.addData("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnLeftWithPID(double angle) throws InterruptedException
    {
        //calibration constants
        double p = .004; double i = .000035; //double d = 2.0;
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long lastTime = System.currentTimeMillis();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            error = angle - Math.abs(getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime/(error-pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if(output < .15)
                output = 0;
            //+ (reset * i) + derivative
            turnLeft(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            telemetry.addData("derivative", derivative);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        if(Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.addData("turn", "success");
        else
            telemetry.addData("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnLeftWithPID(double angle, double p, double i, double d) throws InterruptedException {
        //calibration constants
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getGyroYaw() - beforeAngle) < angle) {
            error = angle - Math.abs(getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime / (error - pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if (output < .15)
                output = 0;
            //+ (reset * i) + derivative
            turnLeft(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        if (Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.addData("turn", "success");
        else
            telemetry.addData("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnLeftWithPIDOneSide(double angle, double p, double i, double d) throws InterruptedException {
        //calibration constants
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getGyroYaw() - beforeAngle) < angle) {
            error = angle - Math.abs(getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime / (error - pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if (output < .15)
                output = 0;
            else if(output > 1)
                output = 1;
            //+ (reset * i) + derivative
//            FL.setPower(-output);
//            BL.setPower(-output);
            FR.setPower(output);
            BR.setPower(output);
            telemetry.log().add("output", output);
            telemetry.log().add("proportion", proportional);
            telemetry.log().add("reset", reset * i);
            telemetry.log().add("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = getGyroYaw();
        telemetry.log().add("afterYawAngle", beforeAngle);
        if (Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.log().add("turn", "success");
        else
            telemetry.log().add("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void turnRightWithPIDOneSide(double angle, double p, double i, double d) throws InterruptedException {
        //calibration constants
        double error = angle;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        long lastTime = System.currentTimeMillis();
        long firstTime = System.currentTimeMillis();
        while (Math.abs(getGyroYaw() - beforeAngle) < angle && Math.abs(System.currentTimeMillis() - firstTime) < 3000) {
            error = angle - Math.abs(getGyroYaw() - beforeAngle);
            //proportional
            proportional = error * p;
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            derivative = deltaTime / (error - pastError); //(error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            //Range.clip(output, -1, 1);
            if (output < .15)
                output = 0;
            else if(output > 1)
                output = 1;
            //+ (reset * i) + derivative
            FR.setPower(output);
            BR.setPower(output);
            telemetry.log().add("output", output);
            telemetry.log().add("proportion", proportional);
            telemetry.log().add("reset", reset * i);
            telemetry.log().add("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        double afterAngle = getGyroYaw();
        telemetry.log().add("afterYawAngle", beforeAngle);
        if (Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
            telemetry.log().add("turn", "success");
        else
            telemetry.log().add("turn", "failure");
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    //gyro stabilization
    public void moveForward(double power, int distance) throws InterruptedException {
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = CORRECTION;
        long lastTime = System.nanoTime();
        double signedDifference;
        while (Math.abs(getAvg() - beforeALV) < distance) {
            FR.setPower(power);
            BR.setPower(power);
            FL.setPower(-power);
            BL.setPower(-power);
            double difference = Math.abs(getGyroYaw() - beforeAngle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                difference = Math.abs(getGyroYaw() - beforeAngle);
                if(getGyroYaw() < beforeAngle) {
                    FR.setPower(power * (1 + difference * correction));
                    BR.setPower(power * (1 + difference * correction));
                    FL.setPower(-power * (1 - (1 + difference * correction)));
                    BL.setPower(-power * (1 - (1 + difference * correction)));
                }
                else if(getGyroYaw() > beforeAngle) {
                    FR.setPower(power * (1 - (1 + difference * correction)));
                    BR.setPower(power * (1 - (1 + difference * correction)));
                    FL.setPower(-power * (1 + difference * correction));
                    BL.setPower(-power  * (1 + difference * correction));
                }
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
        else {
            telemetry.addData("success", "correction failed");
            telemetry.update();
        }
    }

    //gyro stabilization
    public void moveForward(double power, int distance, double angle) throws InterruptedException {
        beforeALV = getAvg();
        double correction = .14;
        long lastTime = System.nanoTime();
        double signedDifference;
        while (Math.abs(getAvg() - beforeALV) < distance) {
            FR.setPower(power);
            BR.setPower(power);
            FL.setPower(-power);
            BL.setPower(-power);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < angle) {
                    FR.setPower(power * (1 + difference * correction));
                    BR.setPower(power * (1 + difference * correction));
                    FL.setPower(-power * (1 - difference * correction));
                    BL.setPower(-power * (1 - difference * correction));
                }
                else if(getGyroYaw() > angle) {
                    FR.setPower(power * (1 - (difference * correction)));
                    BR.setPower(power * (1 - (difference * correction)));
                    FL.setPower(-power * (1 + difference * correction));
                    BL.setPower(-power  * (1 + difference * correction));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
            }
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
        else {
            telemetry.addData("success", "correction failed");
            telemetry.update();
        }

    }

    public void moveForwardToPush(double power, int distance, double angle) throws InterruptedException {
        beforeALV = getAvg();
        double correction = .25;
        long lastTime = System.nanoTime();
        double signedDifference;
        while (Math.abs(getAvg() - beforeALV) < distance) {
            FR.setPower(power);
            BR.setPower(power);
            FL.setPower(-power);
            BL.setPower(-power);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < angle) {
                    FR.setPower(power * (1 + difference * correction));
                    BR.setPower(power * (1 + difference * correction));
                    FL.setPower(-power * (1 - difference * correction));
                    BL.setPower(-power * (1 - difference * correction));
                }
                else if(getGyroYaw() > angle) {
                    FR.setPower(power * (1 - (difference * correction)));
                    BR.setPower(power * (1 - (difference * correction)));
                    FL.setPower(-power * (1 + difference * correction));
                    BL.setPower(-power  * (1 + difference * correction));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
            }
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
        else {
            telemetry.addData("success", "correction failed");
            telemetry.update();
        }

    }

    public void moveBackWardWithCorrection(double power, int distance, double angle) throws InterruptedException {
        moveForward(-power, distance, angle);
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    //gyro stabilization with PID
    public void moveForwardPID(int distance) throws InterruptedException {
        //calibration constants
        double p = .00015; double i = .00000015; //double d = 2.0;
        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = .2;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 150;
        distance += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            if(output < .05)
                output = 0;
            moveForward(output);
            double difference = Math.abs(getGyroYaw() - beforeAngle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < beforeAngle) {
                    FR.setPower(output * (1 + difference * correction));
                    BR.setPower(output * (1 + difference * correction));
                    FL.setPower(-output * (1 - difference * correction));
                    BL.setPower(-output * (1 - difference * correction));
                }
                else if(getGyroYaw() > beforeAngle) {
                    FR.setPower(output * (1 - (difference * correction)));
                    BR.setPower(output * (1 - (difference * correction)));
                    FL.setPower(-output * (1 + difference * correction));
                    BL.setPower(-output  * (1 + difference * correction));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
            }
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }

    //gyro stabilization with PID
    public void moveForwardPID(int distance, double angle) throws InterruptedException {
        //.00025, .00000003, 0.0, 4000
        //calibration constants
        double p = .000275; double i = .000000275; //double d = 2.0;

        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        double correctionLeft = .05;
        double correctionRight = .05;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 200;
        distance += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < angle) {
                    FR.setPower(output * (1 + difference * correctionLeft));
                    BR.setPower(output * (1 + difference * correctionLeft));
                    FL.setPower(-output * (1 - difference * correctionLeft));
                    BL.setPower(-output * (1 - difference * correctionLeft));
                }
                //clockwise turns over correct bc the bad wheel is on the right side.
                else if(getGyroYaw() > angle) {
                    FR.setPower(output * (1 - (difference * correctionRight)));
                    BR.setPower(output * (1 - (difference * correctionRight)));
                    FL.setPower(-output * (1 + difference * correctionRight));
                    BL.setPower(-output  * (1 + difference * correctionRight));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
                difference = Math.abs(getGyroYaw() - angle);
            }
            else {
                if (output < .05)
                    output = 0;
                moveForward(output);
            }
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }

    //gyro stabilization with PID
    /* public void moveForward(int distance, double angle) throws InterruptedException {
        //.00025, .00000003, 0.0, 4000
        //calibration constants
        double p = .0002; double i = .00000015; //double d = 2.0;
        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        double correctionLeft = .0525;
        double correctionRight = .025;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 200;
        distance += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < angle) {
                    FR.setPower(-output * (1 + difference * correctionLeft));
                    BR.setPower(-output * (1 + difference * correctionLeft));
                    FL.setPower(output * (1 - difference * correctionLeft));
                    BL.setPower(output * (1 - difference * correctionLeft));
                }
                //clockwise turns over correct bc the bad wheel is on the right side.
                else if(getGyroYaw() > angle) {
                    FR.setPower(-output * (1 - (difference * correctionRight)));
                    BR.setPower(-output * (1 - (difference * correctionRight)));
                    FL.setPower(output * (1 + difference * correctionRight));
                    BL.setPower(output  * (1 + difference * correctionRight));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
                difference = Math.abs(getGyroYaw() - angle);
            }
            else {
                if (output < .05)
                    output = 0;
                moveBackward(output);
            }
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    } */



    //this contains adjustable constants
    public void moveForwardPID(double p, double i, double d, int distance) throws InterruptedException{
        //.00025, .00000003, 0.0, 4000
        //calibration constants
        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = CORRECTION;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 200;
        distance += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            if(output < .05)
                output = 0;
            moveForward(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }


    public void moveBackwardPID(int distance) throws InterruptedException
    {
        //calibration constants
        double p = .0002; double i = .00000015; //double d = 2.0;
        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = CORRECTION;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            if(output < .05)
                output = 0;
            moveBackward(output);
            //double difference = Math.abs(getGyroYaw() - beforeAngle);
//            while (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
//                if(getGyroYaw() < beforeAngle) {
//                    FR.setPower(output * (1 + difference * correction));
//                    BR.setPower(output * (1 + difference * correction));
//                    FL.setPower(-output);
//                    BL.setPower(-output);
//                }
//                else if(getGyroYaw() > beforeAngle) {
//                    FR.setPower(output);
//                    BR.setPower(output);
//                    FL.setPower(-output * (1 + difference * correction));
//                    BL.setPower(-output  * (1 + difference * correction));
//                }
//                telemetry.addData("LeftPower", FR.getPower());
//                telemetry.addData("RightPower", BR.getPower());
//                telemetry.update();
//                difference = Math.abs(getGyroYaw() - beforeAngle);
//                idle();
//            }
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }

    //this contains adjustable constants
    public void moveBackwardPID(double p, double i, double d, int distance) throws InterruptedException {
        //calibration constants
        double error = distance;
        double pastError = 0.0;
        double output = 0;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double correction = CORRECTION;
        long lastTime = System.currentTimeMillis();
        double difference = Math.abs(getGyroYaw() - beforeAngle);
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            if(output < .05)
                output = 0;
            moveBackward(output);
            difference = Math.abs(getGyroYaw() - beforeAngle);
            while(difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if (getGyroYaw() > beforeAngle) {
                    FR.setPower(output * (1 + difference * correction));
                    BR.setPower(output * (1 + difference * correction));
                    FL.setPower(-output * ((1 - (1 + difference * correction))));
                    BL.setPower(-output * ((1 - (1 + difference * correction))));
                }
                else if (getGyroYaw() < beforeAngle) {
                    FR.setPower(output * ((1 - (1 + difference * correction))));
                    BR.setPower(output * ((1 - (1 + difference * correction))));
                    FL.setPower(-output * (1 + difference * correction));
                    BL.setPower(-output * (1 + difference * correction));
                }
                telemetry.addData("LeftPower", FR.getPower());
                telemetry.addData("RightPower", BR.getPower());
                telemetry.update();
                difference = Math.abs(getGyroYaw() - beforeAngle);
                idle();
            }
        }
        telemetry.addData("output", output);
        telemetry.addData("proportion", proportional);
        telemetry.addData("reset", reset * i);
        //telemetry.addData("derivative", derivative * d);
        telemetry.update();
        pastError = error;
        lastTime = System.currentTimeMillis();
        idle();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }

    //beacon pushing methods
    public void moveBackwardsToWhiteLine(int distance) throws InterruptedException {
        //calibration constants
        double p = .00000625;
        double i = .0000000031; //double d = .00000000002;
        double error = distance;
        double pastError = 0.0;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        double output;
        beforeALV = getAvg();
        double correction = CORRECTION;
        long lastTime = System.currentTimeMillis();
        long firstTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 8 && System.currentTimeMillis() - firstTime < 4000) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            if (output < .05) {
                output = 0;
            }
            FR.setPower(output * .9);
            BR.setPower(output * .9);
            FL.setPower(-output * 1.1);
            BL.setPower(-output * 1.1);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    //beacon pushing methods
    public void moveForwardsToWhiteLine(int distance, double angle) throws InterruptedException {
        //calibration constants
        double p = .00000625; double i = .0000000031; //double d = .00000000002;
        double error = distance;
        double pastError = 0.0;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        double output;
        beforeALV = getAvg();
        double correction = CORRECTION;
        long lastTime = System.currentTimeMillis();
        long firstTime =  System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 8 && System.currentTimeMillis() - firstTime < 4000) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
                if(getGyroYaw() < angle) {
                    FR.setPower(output * (1 + difference * correction));
                    BR.setPower(output * (1 + difference * correction));
                    FL.setPower(-output * (1 - difference * correction));
                    BL.setPower(-output * (1 - difference * correction));
                }
                else if(getGyroYaw() > angle) {
                    FR.setPower(output * (1 - (difference * correction)));
                    BR.setPower(output * (1 - (difference * correction)));
                    FL.setPower(-output * (1 + difference * correction));
                    BL.setPower(-output  * (1 + difference * correction));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
            }
            else if(output < .05) {
                output = 0;
            }
                moveForward(output);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        error = 200;
        pastError = 0.0;
        proportional = 0.0;
        reset = 0.0;
        derivative = 0.0;
        p = .0001;
        reset = 0;
        lastTime = System.currentTimeMillis();
//        firstTime =  System.currentTimeMillis();
//        while (Math.abs(getAvg() - beforeALV) < 200 && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 10 && System.currentTimeMillis() - firstTime < 2000) {
//            moveForward(.15);
//            error = 200 - Math.abs(getAvg() - beforeALV);
//            //proportional
//            proportional = error * p;
//            //integral
//            deltaTime = System.currentTimeMillis() - lastTime;
//            //integral
//            reset += (error * deltaTime);
//            //derivative
//            //derivative = d * (error - pastError)/deltaTime;
//            //output
//            output = proportional + (reset * i);
//            double difference = Math.abs(getGyroYaw() - angle);
//            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
//                if(getGyroYaw() < angle) {
//                    FR.setPower(output * (1 + difference * correction));
//                    BR.setPower(output * (1 + difference * correction));
//                    FL.setPower(-output * (1 - difference * correction));
//                    BL.setPower(-output * (1 - difference * correction));
//                }
//                else if(getGyroYaw() > angle) {
//                    FR.setPower(output * (1 - (difference * correction)));
//                    BR.setPower(output * (1 - (difference * correction)));
//                    FL.setPower(-output * (1 + difference * correction));
//                    BL.setPower(-output  * (1 + difference * correction));
//                }
//                telemetry.addData("LeftPower", FL.getPower());
//                telemetry.addData("RightPower", FR.getPower());
//                telemetry.update();
//                idle();
//            }
//            else
//            if(output < .05)
//                output = 0;
//            moveForward(output);
//            idle();
    }

//    public void moveExtra(double power, int distance) throws InterruptedException{
//        if (Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 8) {
//            moveForwardWithEncoders(power, distance);
//        }
//    }

//    public void turnIntoWhiteLine(double angle) throws InterruptedException {
//        //calibration constants
//        double p = .006; double i = .00004; double d = 0.0;
//        //double p = .004; double i = .000015;
//        double error = angle;
//        double pastError = 0.0;
//        double output;
//        double proportional = 0.0;
//        double reset = 0.0;
//        double derivative = 0.0;
//        double deltaTime;
//        beforeAngle = getGyroYaw();
//        telemetry.addData("beforeYawAngle", beforeAngle);
//        telemetry.update();
//        long firstTime = System.currentTimeMillis();
//        long lastTime = System.currentTimeMillis();
//        while (Math.abs(getGyroYaw() - beforeAngle) < (angle) && Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && System.currentTimeMillis() - firstTime < 3000) {
//            error = angle - Math.abs(getGyroYaw() - beforeAngle);
//            //proportional
//            proportional = error * p;
//            deltaTime = System.currentTimeMillis() - lastTime;
//            //integral
//            reset += (error * deltaTime);
//            //derivative
//            derivative = deltaTime / (error - pastError); //(error - pastError)/deltaTime;
//            //output
//            output = proportional + (reset * i);
//            //Range.clip(output, -1, 1);
//            if (output < .15)
//                output = 0;
//            else if(output > 1)
//                output = 1;
//            //+ (reset * i) + derivative
//            turnRight(output);
//            telemetry.log().add("output", output);
//            telemetry.log().add("proportion", proportional);
//            telemetry.log().add("reset", reset * i);
//            telemetry.log().add("derivative", derivative * d);
//            telemetry.update();
//            pastError = error;
//            lastTime = System.currentTimeMillis();
//            idle();
//        }
//        double afterAngle = getGyroYaw();
//        telemetry.log().add("afterYawAngle", beforeAngle);
//        if (Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
//            telemetry.log().add("turn", "success");
//        else
//            telemetry.log().add("turn", "failure");
//        telemetry.update();
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//    }

//    public void turnIntoWhiteLineRed(double angle) throws InterruptedException {
//        //calibration constants
//        double p = .00375; double i = .000018; double d = 0.0;
//        //double p = .004; double i = .000015;
//        double error = angle;
//        double pastError = 0.0;
//        double output;
//        double proportional = 0.0;
//        double reset = 0.0;
//        double derivative = 0.0;
//        double deltaTime;
//        beforeAngle = getGyroYaw();
//        telemetry.addData("beforeYawAngle", beforeAngle);
//        telemetry.update();
//        long firstTime = System.currentTimeMillis();
//        long lastTime = System.currentTimeMillis();
//        while (Math.abs(getGyroYaw() - beforeAngle) < (angle - 6) && Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10 && System.currentTimeMillis() - firstTime < 3000) {
//            error = angle - Math.abs(getGyroYaw() - beforeAngle);
//            //proportional
//            proportional = error * p;
//            deltaTime = System.currentTimeMillis() - lastTime;
//            //integral
//            reset += (error * deltaTime);
//            //derivative
//            derivative = deltaTime / (error - pastError); //(error - pastError)/deltaTime;
//            //output
//            output = proportional + (reset * i);
//            //Range.clip(output, -1, 1);
//            if (output < .15)
//                output = 0;
//            else if(output > 1)
//                output = 1;
//            //+ (reset * i) + derivative
//            turnLeft(output);
//            telemetry.log().add("output", output);
//            telemetry.log().add("proportion", proportional);
//            telemetry.log().add("reset", reset * i);
//            telemetry.log().add("derivative", derivative * d);
//            telemetry.update();
//            pastError = error;
//            lastTime = System.currentTimeMillis();
//            idle();
//        }
//        double afterAngle = getGyroYaw();
//        telemetry.log().add("afterYawAngle", beforeAngle);
//        if (Math.abs(afterAngle - beforeAngle) > angle - 1 && Math.abs(afterAngle - beforeAngle) < angle + 1)
//            telemetry.log().add("turn", "success");
//        else
//            telemetry.log().add("turn", "failure");
//        telemetry.update();
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//    }
//
//    //beacon pushing methods
//    public void moveBackwardsToWhiteLine(int distance, double power, double angle) throws InterruptedException {
//        //calibration constants
//        double p = .00015; double i = .00000015; //double d = .00000000002;
//        beforeALV = getAvg();
//        beforeAngle = angle;
//        double correction = .07;
//        long lastTime = System.currentTimeMillis();
//        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10) {
//            double difference = Math.abs(getGyroYaw() - angle);
//            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
//                if(getGyroYaw() < angle) {
//                    FR.setPower(-power * (1 + difference * correction));
//                    BR.setPower(-power * (1 + difference * correction));
//                    FL.setPower(power * (1 - difference * correction));
//                    BL.setPower(power * (1 - difference * correction));
//                }
//                else if(getGyroYaw() > angle) {
//                    FR.setPower(-power * (1 - (difference * correction)));
//                    BR.setPower(-power * (1 - (difference * correction)));
//                    FL.setPower(power * (1 + difference * correction));
//                    BL.setPower(power  * (1 + difference * correction));
//                }
//                telemetry.addData("LeftPower", FL.getPower());
//                telemetry.addData("RightPower", FR.getPower());
//                telemetry.update();
//            }
//            else
//                moveBackward(power);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//    }

    //beacon pushing methods ///IMPORTANT NOTE - CHANGED THIS TO STOP AT FIRST BEACON
    public void moveToWallRed(int distance, double power) throws InterruptedException {
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double output = power * basePowerMultiplier();
        double startTime = System.currentTimeMillis();
//        deltaT ime / (getAvg() - pastError);
        while (Math.abs(getGyroYaw() - beforeAngle) < 25 && Math.abs(System.currentTimeMillis() - startTime) < 4000) {{
                FR.setPower(output * .9);
                BR.setPower(output * .9);
                FL.setPower(-output * 1.2);
                BL.setPower(-output * 1.2);
                idle();
            }
        }
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(System.currentTimeMillis() - startTime) < 4000) {
            FR.setPower(output * .75);
            BR.setPower(output * .75);
            FL.setPower(-output * .9);
            BL.setPower(-output * .9);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
//        output = power * .7;
//        while (Math.abs(getAvg() - beforeALV) < (distance * .25)) {
//            FR.setPower(output * 1.5);
//            BR.setPower(output * 1.5);
//            FL.setPower(-output * .8);
//            BL.setPower(-output * .8);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
    }

    public void moveToWallRed_Stop(int distance, double power, double light) throws InterruptedException {
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double output = power * basePowerMultiplier();
        double startTime = System.currentTimeMillis();
//        deltaT ime / (getAvg() - pastError);
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(System.currentTimeMillis() - startTime) < 10000 && colorSensorAverageValues(colorSensorWLA) < light) {
            FR.setPower(output * .9);
            BR.setPower(output * .9);
            FL.setPower(-output * 1.1);
            BL.setPower(-output * 1.1);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void resetEncoders() throws InterruptedException {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Positive values power values are towards the wall.
    public void moveToWallBlue(int distance, double power) throws InterruptedException {
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double output = power * basePowerMultiplier();
        //Math.abs(getAvg() - beforeALV) < distance
        while (Math.abs(getGyroYaw() - beforeAngle) < 17 && Math.abs(getAvg() - beforeALV) < distance) {
            FR.setPower(-output * 1.15);
            BR.setPower(-output * 1.15);
            FL.setPower(output * .9);
            BL.setPower(output * .9);
            idle();
        }
        output = .25;
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 11) {
            FR.setPower(-output * 1.05);
            BR.setPower(-output * 1.05);
            FL.setPower(output * .9);
            BL.setPower(output * .9);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        if(Math.abs(getAvg() - beforeALV) > distance)
            telemetry.addData("hit white line", "success");
         //remember that you added this pause
//        beforeALV = getAvg();
//        while (Math.abs(getAvg() - beforeALV) < 100) {
//            FR.setPower(-output * 1.1);
//            BR.setPower(-output * 1.1);
//            FL.setPower(output * .9);
//            BL.setPower(output * .9);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);

//        beforeALV = getAvg();
//        while (Math.abs(getAvg() - beforeALV) < 50) {
//            FR.setPower(-output * 1.1);
//            BR.setPower(-output * 1.1);
//            FL.setPower(output * .9);
//            BL.setPower(output * .9);
//            idle();
//        }
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
//        output = power * .7;
//        while (Math.abs(getAvg() - beforeALV) < (distance * .25)) {
//            FR.setPower(output * 1.5);
//            BR.setPower(output * 1.5);
//            FL.setPower(-output * .8);
//            BL.setPower(-output * .8);
//            idle();
//        }
    }

    public void moveToSecondLine(int distance, double power, double light) throws InterruptedException {
        beforeALV = getAvg();
        double output = power;
        double starttime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance && colorSensorAverageValues(colorSensorWLA) < light) {
            FR.setPower(-output * .9);
            BR.setPower(-output * .9);
            FL.setPower(output * 1.1);
            BL.setPower(output * 1.1);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }


    public void moveBackToWhiteLine(int distance, double power) throws InterruptedException {
        beforeALV = getAvg();
        while (Math.abs(getAvg() - beforeALV) < distance && colorSensorAverageValues(colorSensorWLA) < 12) {
            FR.setPower(-power * .9);
            BR.setPower(-power * .9);
            FL.setPower(power * 1.1);
            BL.setPower(power * 1.1);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveBackToWhiteLine(int distance, double power, double accuracy) throws InterruptedException {
        beforeALV = getAvg();
        while (Math.abs(getAvg() - beforeALV) < distance && colorSensorAverageValues(colorSensorWLA) < accuracy) {
            FR.setPower(-power * .9);
            BR.setPower(-power * .9);
            FL.setPower(power * 1.1);
            BL.setPower(power * 1.1);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public boolean onWhiteLine() throws InterruptedException {
        return (colorSensorAverageValues(colorSensorWLA) > 10);
    }
    public boolean onWhiteLine(double accuracy) throws InterruptedException {
        return (colorSensorAverageValues(colorSensorWLA) >= accuracy);
    }

    //beacon pushing methods
    public void moveBackwardsToWhiteLineRed(int distance, double power, double angle) throws InterruptedException {
        //calibration constants
        double p = .00015; double i = .00000015; //double d = .00000000002;
        beforeALV = getAvg();
        beforeAngle = angle;
        double correction = .07;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 10) {
//            double difference = Math.abs(getGyroYaw() - angle);
//            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
//                if(getGyroYaw() < angle) {
//                    FR.setPower(power * (1 + difference * correction));
//                    BR.setPower(power * (1 + difference * correction));
//                    FL.setPower(-power * (1 - difference * correction));
//                    BL.setPower(-power * (1 - difference * correction));
//                }
//                else if(getGyroYaw() > angle) {
//                    FR.setPower(power * (1 - (difference * correction)));
//                    BR.setPower(power * (1 - (difference * correction)));
//                    FL.setPower(-power * (1 + difference * correction));
//                    BL.setPower(-power  * (1 + difference * correction));
//                }
//                telemetry.addData("LeftPower", FL.getPower());
//                telemetry.addData("RightPower", FR.getPower());
//                telemetry.update();
//            }
//            else
//                moveBackward(power);
//            idle();
            FR.setPower(-power);
            BR.setPower(-power);
            FL.setPower(power * 1.2);
            BL.setPower(power * 1.2);
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveForwardsToWhiteLineRed(int distance, double power, double perpendicular) throws InterruptedException {
        //calibration constants
        double p = .00015;
        double i = .00000015; //double d = .00000000002;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double turn = perpendicular - beforeAngle;
        double correction = .07;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 10) {
//            double difference = Math.abs(getGyroYaw() - angle);
//            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
//                if(getGyroYaw() < angle) {
//                    FR.setPower(power * (1 + difference * correction));
//                    BR.setPower(power * (1 + difference * correction));
//                    FL.setPower(-power * (1 - difference * correction));
//                    BL.setPower(-power * (1 - difference * correction));
//                }
//                else if(getGyroYaw() > angle) {
//                    FR.setPower(power * (1 - (difference * correction)));
//                    BR.setPower(power * (1 - (difference * correction)));
//                    FL.setPower(-power * (1 + difference * correction));
//                    BL.setPower(-power  * (1 + difference * correction));
//                }
//                telemetry.addData("LeftPower", FL.getPower());
//                telemetry.addData("RightPower", FR.getPower());
//                telemetry.update();
//            }
//            else
//                moveBackward(power);
//            idle();
            FR.setPower(power * 2.5);
            BR.setPower(power * 2.5);
            FL.setPower(-power * .75);
            BL.setPower(-power * .75);
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    //0 represents blue and 1 represents red
    public int beaconValue(ColorSensor color) throws InterruptedException {
        if(colorSensorBlue(color)  == 255)
            return -1;
        else if(colorSensorBlue(color) > colorSensorRed(color)) {
            telemetry.addData("Beacon", "Blue");
            telemetry.update();
            return 0;
        }
        telemetry.addData("Beacon", "Red");
        telemetry.update();
        return 1;
    }


    //Compares both sensors to see which one has HIGHER of one color. 0 is for back, 1 is for front.
    public int beaconCompareBlue(ColorSensor back, ColorSensor front) throws InterruptedException {
        if (colorSensorBlue(back) > colorSensorBlue(front)) return 1;
        return 0;
    }

    public int beaconCompareRed() throws InterruptedException {
        if(colorSensorRed(backBeacon) > colorSensorRed(frontBeacon)) return 1;
        return 0;
    }

    public void pushRedBeacon() throws InterruptedException {
        //power: .15
        //distance: 25
        //move forward and push the correct beacon
        if (beaconValue(backBeacon) == 255) {
            if (beaconValue(frontBeacon) == 1)
                moveBackBeacon();
            else
                moveFrontBeacon();
        }
        else if (beaconValue(frontBeacon) == 255) {
            if (beaconValue(backBeacon) == 1)
                moveFrontBeacon();
            else
                moveBackBeacon();
        }
        else if (beaconValue(backBeacon) == 1) {
            moveFrontBeacon();
        }
        else if (beaconValue(frontBeacon) == 1) {
            moveBackBeacon();
        }
        telemetry.addData("red", "hit");
        telemetry.update();
    }

    public void resetCount() throws InterruptedException {
        count = 0;
    }

    public void pushBlueBeacon() throws InterruptedException {
//        count += 1;
//        if(Math.abs(colorSensorAverageValues(colorSensorWLA) - whiteACV) > 10) {
//            if (beaconCompareBlue(backBeacon,frontBeacon) == 0) {
//                moveBackBeacon();
//            }
//            else if (beaconCompareBlue(backBeacon,frontBeacon) == 1) {
//                moveFrontBeacon();
//            }
//        }
//        if (beaconValue(backBeacon) == 0) {
//            moveFrontBeacon();
//        }
//        else if (beaconValue(frontBeacon) == 0) {
//            moveBackBeacon();
//        }
        int count = 0;
        if (beaconValue(backBeacon) == 255) {
            if (beaconValue(frontBeacon) == 0)
                moveBackBeacon();
            else
                moveFrontBeacon();
        } else if (beaconValue(frontBeacon) == 255) {
            if (beaconValue(backBeacon) == 0)
                moveFrontBeacon();
            else
                moveBackBeacon();
        } else if (beaconValue(backBeacon) == 0) {
            moveFrontBeacon();
        } else if (beaconValue(frontBeacon) == 0) {
            moveBackBeacon();
        }
        telemetry.addData("blue", "hit");
        telemetry.update();


    }
//        else if(count < 2) {
//            moveBackToWhiteLine(200, .125);
//            pushBlueBeacon();
//        }
//        else {
//            moveBackToWhiteLine(200, -.125);
//            pushBlueBeacon();
//        }
//        telemetry.addData("blue", "hit");
//        telemetry.update();

    public void correctBlue() throws InterruptedException{

    }

    //test methods

    //tests to see whether turning left or turning right is negative or postive
    public void testTurningNegative(double power, int angle) throws InterruptedException
    {
        double beforeAngle = getGyroYaw();
        turnRightWithGyro(power, angle);
        sleep(1000);
        double finalAngle = getGyroYaw();
        telemetry.addData("turnRightResults", (finalAngle - beforeAngle));
        telemetry.update();
        sleep(5000);
        beforeAngle = getGyroYaw();
        turnLeftWithGyro(power, angle);
        sleep(1000);
        finalAngle = getGyroYaw();
        telemetry.addData("turnLeftResults", (finalAngle - beforeAngle));
        telemetry.update();
        sleep(5000);
    }

    public void correct(double perpendicular, double p, double i, double d, double underShoot) throws InterruptedException {
        //double p = .004; double i = .000015; //double d = 2.0;
        double angle = Math.abs(perpendicular - getGyroYaw());
        if (perpendicular > getGyroYaw()) {
            turnRightWithPID(angle - underShoot, p, i, d);
        }
        else if(perpendicular < getGyroYaw()) {
            turnLeftWithPID(angle - underShoot, p, i, d);
        }
    }

    public void correctOneSideLeft(double perpendicular, double p, double i, double d, double underShoot) throws InterruptedException {
        //double p = .004; double i = .000015; //double d = 2.0;
        double angle = Math.abs(perpendicular - getGyroYaw());
        turnLeftWithPIDOneSide(angle - underShoot, p, i, d);
    }

    public void correctOneSideRight(double perpendicular, double p, double i, double d, double underShoot) throws InterruptedException {
        //double p = .004; double i = .000015; //double d = 2.0;
        double angle = Math.abs(perpendicular - getGyroYaw());
        turnRightWithPIDOneSide(angle, p, i, d);
    }

    //miscellaneous
    //takes a 45˚ arc turn followed by moving backwards into the cap ball
    public void moveBackwardsWithATiltRight(double power, double distance) throws InterruptedException {
        beforeAngle = getGyroYaw();
        while(Math.abs(getGyroYaw() - beforeAngle) < 45) {
            FR.setPower(-power * .3);
            BR.setPower(-power * .3);
            FL.setPower(power * 1.66);
            BL.setPower(power * 1.66);
            idle();
        }
        beforeALV = getAvg();
        while(Math.abs(getGyroYaw() - getAvg()) < distance) {
            FR.setPower(-power);
            BR.setPower(-power);
            FL.setPower(power);
            BL.setPower(power);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void moveBackwardsWithATiltLeft(double power, double distance) throws InterruptedException{
        beforeAngle = getGyroYaw();
        while(Math.abs(getGyroYaw() - beforeAngle) < 45) {
            FR.setPower(-power * 1.66);
            BR.setPower(-power * 1.66);
            FL.setPower(power * .3);
            BL.setPower(power * .3);
            idle();
        }
        beforeALV = getAvg();
        while(Math.abs(getGyroYaw() - getAvg()) < distance) {
            FR.setPower(-power);
            BR.setPower(-power);
            FL.setPower(power);
            BL.setPower(power);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void nonAutoClear() throws InterruptedException {
        telemetry.log().add("test log");
        telemetry.update();
        telemetry.log().add("next Line");
        telemetry.update();
    }
    public void moveBackBeacon() throws InterruptedException {
        BackBeaconPusher.setPosition(.9);
        Thread.sleep(1000);
        BackBeaconPusher.setPosition(.15);
    }

    public void moveFrontBeacon() throws InterruptedException {
        FrontBeaconPusher.setPosition(.9);
        Thread.sleep(1500);
        FrontBeaconPusher.setPosition(0);
    }

    public void moveManBeaconL() throws InterruptedException {
        ManBeaconL.setPosition(.85);
        sleep(1500);
        ManBeaconL.setPosition(.3);
    }

    public void moveManBeaconR() throws InterruptedException {
        ManBeaconR.setPosition(.25);
        Thread.sleep(1000);
        ManBeaconR.setPosition(.7);
    }

    //Uses gryo to calculate distance from the robot to Parallel from robot closest edge
    //Returns positive value if angled clockwise, negative if angled counterclockwise
    public double getTrigDistance(double parallel) throws InterruptedException {
        double angle = Math.abs(parallel - getGyroYaw());
        if (parallel > getGyroYaw()) {
            return Math.sin(angle) * -42.72; /* 42.72 = length of robot in cm */
        }
        if (parallel < getGyroYaw()) {
            return Math.sin(angle) * 42.72;
        }
        return 0;
    }

    //Uses gryo & range sensors to calculate distance from robot closest edge to Parallel to wall.
    //Returns positive regardless of direction angled.
    public double getDistanceToWall(double parallel) throws InterruptedException {
        double angle = Math.abs(parallel - getGyroYaw());
        if (parallel > getGyroYaw()){
            return Math.cos(angle) * getRawDistance() + getTrigDistance(parallel);
        }
        else if (parallel < getGyroYaw()) {
          return Math.cos(90 - angle) * getRawDistance();
        }
        return 0;
    }

    public double getRawDistance() throws InterruptedException {
        return wallSensor.getDistance(DistanceUnit.CM);
    }

    public void followRedWallBackward(double distance, double wallDistance, double perpendicular) throws InterruptedException {
        //calibration constaints
        double p = .00015; double i = .00000015; //double d = 2.0;
        double error = distance;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        beforeALV = getAvg();
        beforeAngle = getGyroYaw();
        double wallCorrection = .02;
        double angleCorrection = .2;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 150;
        distance += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < distance) {
            error = distance - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            double wallDifference = Math.abs(wallDistance - wallSensor.getDistance(DistanceUnit.CM) - getTrigDistance(perpendicular));
            double angleDifference = Math.abs(perpendicular - getGyroYaw());
            if (wallDifference > 4 || angleDifference < 4) {
                if (wallDifference > 4) {
                    if (wallDistance < Math.abs(wallSensor.getDistance(DistanceUnit.CM) - getTrigDistance(perpendicular))) {
                        FR.setPower(-output * (1 + wallDifference * wallCorrection));
                        BR.setPower(-output * (1 + wallDifference * wallCorrection));
                        FL.setPower(output * (1 - wallDifference * wallCorrection));
                        BL.setPower(output * (1 - wallDifference * wallCorrection));
                    }
                    else if (wallDistance > Math.abs(wallSensor.getDistance(DistanceUnit.CM) - getTrigDistance(perpendicular))) {
                        FR.setPower(-output * (1 - wallDifference * wallCorrection));
                        BR.setPower(-output * (1 - wallDifference * wallCorrection));
                        FL.setPower(output * (1 + wallDifference * wallCorrection));
                        BL.setPower(output * (1 + wallDifference * wallCorrection));
                    }
                    wallDifference = Math.abs(wallDistance - wallSensor.getDistance(DistanceUnit.CM) - getTrigDistance(perpendicular));
                }
                else if (angleDifference > 4) {
                    if (getGyroYaw() < beforeAngle) {
                        FR.setPower(-output * (1 + angleDifference * angleCorrection));
                        BR.setPower(-output * (1 + angleDifference * angleCorrection));
                        FL.setPower(output * (1 - angleDifference * angleCorrection));
                        BL.setPower(output * (1 - angleDifference * angleCorrection));
                    }
                    else if (getGyroYaw() > beforeAngle) {
                        FR.setPower(-output * (1 - angleDifference * angleCorrection));
                        BR.setPower(-output * (1 - angleDifference * angleCorrection));
                        FL.setPower(output * (1 + angleDifference * angleCorrection));
                        BL.setPower(output * (1 + angleDifference * angleCorrection));
                    }
                    angleDifference = Math.abs(perpendicular - getGyroYaw());
                }
                telemetry.addData("LeftPower", FR.getPower());
                telemetry.addData("RightPower", BR.getPower());
                telemetry.update();
                idle();
            }
            if(output < .05)
                output = 0;
            moveForward(output);
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void arcTurnRight (double power, double angle) throws InterruptedException {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            FR.setPower(power * .5);
            BR.setPower(power * .5);
            FL.setPower(-power);
            FL.setPower(-power);
            idle();
        }
        beforeAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    public void arcTurnLeft (double power, double angle) throws InterruptedException {
        beforeAngle = getGyroYaw();
        telemetry.addData("beforeYawAngle", beforeAngle);
        telemetry.update();
        while(Math.abs(getGyroYaw() - beforeAngle) < angle) {
            FR.setPower(power);
            BR.setPower(power);
            FL.setPower(-power * .5);
            FL.setPower(-power * .5);
            idle();
        }
        beforeAngle = getGyroYaw();
        telemetry.addData("afterYawAngle", beforeAngle);
        telemetry.update();
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

/** ================== WORLD's COMP. METHODS =================== **/

    //MoveForwardPID() + additional limit to movement (stop moving once range sensor detects close enough to wall
    public void moveStartToWall (int encoderCAP, double angle /* int rangeDistance, double parallel */) throws InterruptedException
    {
        double p = .000275; double i = .000000275; //double d = 2.0;
        double error = encoderCAP;
        double pastError = 0.0;
        double output;
        double proportional = 0.0;
        double reset = 0.0;
        double derivative = 0.0;
        double deltaTime;
        int angleError;
        beforeALV = getAvg();
        double correctionLeft = .05;
        double correctionRight = .05;
        double voltageAverage = (hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage() + hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage())/2;;
        double change = (13.5 - voltageAverage) * 200;
        encoderCAP += change;
        long lastTime = System.currentTimeMillis();
        while (Math.abs(getAvg() - beforeALV) < encoderCAP /* && rangeDistance > getDistanceToWall(parallel) */ ) {
            error = encoderCAP - Math.abs(getAvg() - beforeALV);
            //proportional
            proportional = error * p;
            //integral
            deltaTime = System.currentTimeMillis() - lastTime;
            //integral
            reset += (error * deltaTime);
            //derivative
            //derivative = d * (error - pastError)/deltaTime;
            //output
            output = proportional + (reset * i);
            double difference = Math.abs(getGyroYaw() - angle);
            if (difference > 2 && Math.abs(getAvg() - beforeALV) < encoderCAP) {
                if(getGyroYaw() < angle) {
                    FR.setPower(output * (1 + difference * correctionLeft));
                    BR.setPower(output * (1 + difference * correctionLeft));
                    FL.setPower(-output * (1 - difference * correctionLeft));
                    BL.setPower(-output * (1 - difference * correctionLeft));
                }
                //clockwise turns over correct bc the bad wheel is on the right side.
                else if(getGyroYaw() > angle) {
                    FR.setPower(output * (1 - (difference * correctionRight)));
                    BR.setPower(output * (1 - (difference * correctionRight)));
                    FL.setPower(-output * (1 + difference * correctionRight));
                    BL.setPower(-output  * (1 + difference * correctionRight));
                }
                telemetry.addData("LeftPower", FL.getPower());
                telemetry.addData("RightPower", FR.getPower());
                telemetry.update();
                idle();
                difference = Math.abs(getGyroYaw() - angle);
            }
            else {
                if (output < .05)
                    output = 0;
                moveForward(output);
            }
            telemetry.addData("output", output);
            telemetry.addData("proportion", proportional);
            telemetry.addData("reset", reset * i);
            //telemetry.addData("derivative", derivative * d);
            telemetry.update();
            pastError = error;
            lastTime = System.currentTimeMillis();
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("EncoderMovement", Math.abs(getAvg() - beforeALV));
        if (Math.abs(beforeAngle - getGyroYaw()) < 2)
            telemetry.addData("success", "correction works");
        else
            telemetry.addData("failure", "correction failed");
        if(error < -20 && error > 20)
            telemetry.addData("success", "PID works");
        else
            telemetry.addData("failure", "PID failed");
        telemetry.update();
    }










/* =========== DISCARDED CODE ====================




 public void maintainWallDistance(double power, double distance) throws InterruptedException{
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
        beforeALV = getAvg();
        double beforeDistance = getDistance(rangeSensorRed);
        double difference = 0;
        while(Math.abs(getAvg() - beforeALV) < distance) {
            if(getDistance(rangeSensorRed) - beforeDistance < -2) {
                difference = Math.abs(getDistance(rangeSensorRed) - beforeDistance);
                FR.setPower(power * (1 + .5 * difference));
                BR.setPower(power * (1 + .5 * difference));
                FL.setPower(-power);
                BL.setPower(-power);
            }
            else if(getDistance(rangeSensorRed) - beforeDistance > 2) {
                difference = Math.abs(getDistance(rangeSensorRed) - beforeDistance);
                FR.setPower(power);
                BR.setPower(power);
                FL.setPower(-power * (1 + .5 * difference));
                BL.setPower(-power * (1 + .5 * difference));
            }
            else {
                FR.setPower(power);
                BR.setPower(power);
                FL.setPower(-power);
                BL.setPower(-power);
            }
            idle();
        }
    }

        public void pushFrontRed(double angle) throws InterruptedException {
//        double dist = getDist(rangeSensor);
//        while (opModeIsActive() && (dist > 13 || dist < 6)) {
//            if (dist < 6)
//                moveBackward(.15);
//            else if (dist > 13)
//                moveForward(.15);
//            else{
//
//            }
//            dist = getDist(rangeSensor);
//        }
//        if(beaconValue(colorSensorBeacon) == 1)
//            moveManBeaconR();
//        else
//            moveManBeaconL();
//        Thread.sleep(500);
//        if(beaconValue(colorSensorBeacon) != 1) {
//            Thread.sleep(5000);
//            moveManBeaconR();
//        }
//        correct(angle, .01, .00025, 0, 0);
        double dist = getDist(rangeSensor);
        if(dist > 10.5)
            moveForward(.175, 8, angle);
        dist = getDist(rangeSensor);
        telemetry.addData("distance", dist);
        telemetry.update();
        beforeALV = getAvg();
        //REMOVE AFTER TESTING
        //sleep(5000);
        while (dist > 9.5 && opModeIsActive() && Math.abs(getAvg() - beforeALV) < 415) {
            moveForward(.14);
            dist = getDist(rangeSensor);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("distance", dist);
        telemetry.update();
        sleep(500);
        //moveForward(.175, 450, angle);
        if(beaconValue(colorSensorBeacon) == 1) {
            ManBeaconL.setPosition(1);
            sleep(1000);
            //moveForwardWithEncoders(.14, 100);
            if(getDist(rangeSensor) > 7)
                mashBeacons(.12, 100);
            ManBeaconL.setPosition(.3);
        }
        else {
            ManBeaconR.setPosition(.15);
            sleep(1000);
            //moveForwardWithEncoders(.14, 100);
            if(getDist(rangeSensor) > 7)
                mashBeacons(.12, 100);
            ManBeaconR.setPosition(.7);
        }
        if(beaconValue(colorSensorBeacon) != 1) {
            moveBackWardWithEncoders(.14, 20);
            sleep(5000);
            ManBeaconL.setPosition(1);
            mashBeacons(.14, 20);
            sleep(1000);
            ManBeaconL.setPosition(.3);
            bringDownShooter(-.4, 800);
            moveBackWardWithEncoders(.6, 2800);
            turnRightWithGyro(.3, 30);
            sleep(30000);
        }
    }

//    bringDownShooter(-.4, 800);
//    moveBackWardWithEncoders(.4, 500);
//    turnRightWithPID(45, .004, .000015, 0.0);
//    //double p = .004; double i = .000015;
//    moveBackWardWithEncoders(.6, 2800);
//    sleep(30000);
    public void pushSecondRed(double angle) throws InterruptedException {
//        double dist = getDist(rangeSensor);
//        while (opModeIsActive() && (dist > 13 || dist < 6)) {
//            if (dist < 6)
//                moveBackward(.15);
//            else if (dist > 13)
//                moveForward(.15);
//            else{
//
//            }
//            dist = getDist(rangeSensor);
//        }
//        if(beaconValue(colorSensorBeacon) == 1)
//            moveManBeaconR();
//        else
//            moveManBeaconL();
//        Thread.sleep(500);
//        if(beaconValue(colorSensorBeacon) != 1) {
//            Thread.sleep(5000);
//            moveManBeaconR();
//        }
//        correct(angle, .01, .00025, 0, 0);
        double dist = getDist(rangeSensor);
        if(dist > 8)
            moveForward(.175, 50, angle);
        dist = getDist(rangeSensor);
        telemetry.addData("distance", dist);
        telemetry.update();
        beforeALV = getAvg();
        //REMOVE AFTER TESTING
        sleep(500);
        while (dist > 7 && opModeIsActive() && Math.abs(getAvg() - beforeALV) < 500) {
            moveForward(.14);
            dist = getDist(rangeSensor);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("distance", dist);
        telemetry.update();
        sleep(500);
        //moveForward(.175, 450, angle);
        if(beaconValue(colorSensorBeacon) == 1) {
            ManBeaconL.setPosition(1);
            sleep(1000);
            //moveForwardWithEncoders(.14, 100);
            if(getDist(rangeSensor) > 6)
                mashBeacons(.12, 100);
            ManBeaconL.setPosition(.3);
        }
        else {
            ManBeaconR.setPosition(.15);
            sleep(1000);
            //moveForwardWithEncoders(.14, 100);
            if(getDist(rangeSensor) > 6)
                mashBeacons(.12, 100);
            ManBeaconR.setPosition(.7);
        }
        if(beaconValue(colorSensorBeacon) != 1) {
            moveBackWardWithEncoders(.14, 20);
            sleep(5000);
            ManBeaconL.setPosition(1);
            mashBeacons(.14, 20);
            sleep(1000);
            ManBeaconL.setPosition(.3);
        }
    }

    public void pushFrontBlue(double angle) throws InterruptedException {
        //double p = .004; double i = .000015; //double d = 2.0;
        //double dist = getDist(rangeSensor);

//        int movement = (int) Math.round(dist - 6) * 30;
//        if(movement > 0)
//            moveForward(.175, movement, angle);
//        else if(movement < 0)
//            moveBackWardWithCorrection(.175, -movement, angle);
        double dist = getDist(rangeSensor);
        if(dist > 8)
            moveForwardToPush(.175, 35, angle);
        dist = getDist(rangeSensor);
        telemetry.addData("distance", dist);
        telemetry.update();
        //REMOVE AFTER TESTING
        sleep(500);
        beforeALV = getAvg();
        dist = getDist(rangeSensor);
        while (dist > 7 && opModeIsActive() && Math.abs(getAvg() - beforeALV) < 515) {
            moveForward(.14);
            dist = getDist(rangeSensor);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("distance", dist);
        telemetry.update();
        //sleep(500);
        //moveForward(.175, 450, angle);
        if(beaconValue(colorSensorBeacon) == 0) {
            ManBeaconL.setPosition(.85);
            sleep(1000);
            if(getDist(rangeSensor) > 5.5)
                mashBeacons(.12, 150);
            ManBeaconL.setPosition(.3);
        }
        else {
            ManBeaconR.setPosition(.25);
            sleep(1000);
            if(getDist(rangeSensor) > 5.5)
                mashBeacons(.12, 150);
            ManBeaconR.setPosition(.7);
        }
        if(beaconValue(colorSensorBeacon) != 0) {
            moveBackWardWithEncoders(.14, 20);
            sleep(5000);
            ManBeaconR.setPosition(.25);
            mashBeacons(.14, 100);
            sleep(500);
            ManBeaconR.setPosition(.7);
            sleep(500);
//            bringDownShooter(-.4, 800);
//            moveBackWardWithEncoders(.6, 3000);
//            sleep(30000);
        }
        telemetry.addData("distance", dist);
        telemetry.update();
    }
//    bringDownShooter(-.4, 800);
//    moveBackWardWithEncoders(.4, 500);
//    turnLeftWithPID(45, .004, .000015, 0.0);
//    moveBackWardWithEncoders(.6, 3000);
//    sleep(30000);
    public void pushSecondBlue(double angle) throws InterruptedException {
        //double p = .004; double i = .000015; //double d = 2.0;
        //double dist = getDist(rangeSensor);

//        int movement = (int) Math.round(dist - 6) * 30;
//        if(movement > 0)
//            moveForward(.175, movement, angle);
//        else if(movement < 0)
//            moveBackWardWithCorrection(.175, -movement, angle);
        double dist = getDist(rangeSensor);
        if(dist > 8)
            moveForward(.175, 50, angle);
        dist = getDist(rangeSensor);
        telemetry.addData("distance", dist);
        telemetry.update();
        //REMOVE AFTER TESTING
        sleep(500);
        beforeALV = getAvg();
        dist = getDist(rangeSensor);
        while (dist > 7 && opModeIsActive() && Math.abs(getAvg() - beforeALV) < 500) {
            moveForward(.14);
            dist = getDist(rangeSensor);
            idle();
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        telemetry.addData("distance", dist);
        telemetry.update();
        sleep(500);
        //moveForward(.175, 450, angle);
        if(beaconValue(colorSensorBeacon) == 0) {
            ManBeaconL.setPosition(1);
            sleep(1000);
            if(getDist(rangeSensor) > 7)
                mashBeacons(.12, 100);
            ManBeaconL.setPosition(.3);
        }
        else {
            ManBeaconR.setPosition(.05);
            sleep(1000);
            if(getDist(rangeSensor) > 6)
                mashBeacons(.12, 100);
            ManBeaconR.setPosition(.7);
        }
        if(beaconValue(colorSensorBeacon) != 0) {
            moveBackWardWithEncoders(.14, 20);
            sleep(5000);
            ManBeaconR.setPosition(1);
            mashBeacons(.14, 20);
            sleep(1000);
            ManBeaconR.setPosition(.3);
            sleep(500);
        }
        telemetry.addData("distance", dist);
        telemetry.update();
    }

    //        while (Math.abs(getAvg() - beforeALV) < distance && Math.abs(colorSensorAverageValues(colorSensorWL) - whiteACV) > 10) {
////            double difference = Math.abs(getGyroYaw() - angle);
////            if (difference > 2 && Math.abs(getAvg() - beforeALV) < distance) {
////                if(getGyroYaw() < angle) {
////                    FR.setPower(power * (1 + difference * correction));
////                    BR.setPower(power * (1 + difference * correction));
////                    FL.setPower(-power * (1 - difference * correction));
////                    BL.setPower(-power * (1 - difference * correction));
////                }
////                else if(getGyroYaw() > angle) {
////                    FR.setPower(power * (1 - (difference * correction)));
////                    BR.setPower(power * (1 - (difference * correction)));
////                    FL.setPower(-power * (1 + difference * correction));
////                    BL.setPower(-power  * (1 + difference * correction));
////                }
////                telemetry.addData("LeftPower", FL.getPower());
////                telemetry.addData("RightPower", FR.getPower());
////                telemetry.update();
////            }
////            else
////                moveBackward(power);
////            idle();
//            FR.setPower(power * 1.5);
//            BR.setPower(power * 1.5);
//            FL.setPower(-power);
//            BL.setPower(-power);
//        }
//
//        FR.setPower(0);
//        BR.setPower(0);
//        FL.setPower(0);
//        BL.setPower(0);
    }

//    public void moveForwardsToWhiteLine(int distance, double power, double angle) throws InterruptedException {
//        moveBackwardsToWhiteLine(distance, -power, angle);
//    }


*/
}
