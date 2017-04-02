package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by sopa on 2/27/17.
 */
public abstract class TrollOpMode extends LinearOpMode {
    DcMotor R;
    DcMotor L;
    ColorSensor front;
    OpticalDistanceSensor floor;
    ModernRoboticsI2cRangeSensor ultra;
    public void initalize() {
        R = hardwareMap.dcMotor.get("R");
        L = hardwareMap.dcMotor.get("L");
        front = hardwareMap.colorSensor.get("floor");
        floor = hardwareMap.opticalDistanceSensor.get("floor");
    }
}
