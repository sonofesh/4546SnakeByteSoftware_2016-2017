package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by sopa on 2/21/17.
 */

@Autonomous(name = "40PointAutoNoDelay", group = "Autonomous")
public class ShootOnlyWOD extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("final", "init");
        telemetry.update();
        double power = .85;
        waitForStart();
        moveForwardWithEncoders(.175, 1100);
        sleep(1000);
        bringDownShooter(.275, 1150);
        sleep(1000);
        //Voltage Scaling
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if(voltage <= 13.5)
            power = .95;
        else if(voltage <= 13.75 && voltage > 13.5)
            power = .9;
        else if(voltage > 13.75 && voltage <= 14)
            power = .85;
        else if(voltage > 14)
            power = .8;
        shoot(power, 360);
        sleep(500);
        moveForwardWithEncoders(.4, 2500);
    }
}
