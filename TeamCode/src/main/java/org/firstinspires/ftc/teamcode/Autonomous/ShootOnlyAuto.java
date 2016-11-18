/** 11/17/16
 * ShootOnlyAuto
 * Auto that shoots from start and that's it
 *
 * Viperbots
 * 4546 Snakebyte
 * William Fisher
 *
 */

package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by viperbots 4546 on 11/17/16.
 */
@Autonomous(name = "ShootOnlyAuto", group = "Autonomous")
public class ShootOnlyAuto extends LinearOpMode
{
    DcMotor ShooterB;
    DcMotor ShooterF;
    DcMotor ManLift;
    DcMotor ManIn;
    Servo Stopper;
    Boolean stop;

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

    public void shoot(double power, int distance) throws InterruptedException
    {
        bringDownShooter(.3 * -1, distance);
        sleep(1000);
        ShooterF.setPower(power);
        ShooterB.setPower(-power);
    }

    public void useStopper() throws InterruptedException
    {
        if (!stop) Stopper.setPosition(1);
        else Stopper.setPosition(0);
        sleep(100);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        ShooterB = hardwareMap.dcMotor.get("B");
        ShooterF = hardwareMap.dcMotor.get("F");
        ManLift = hardwareMap.dcMotor.get("ManLift");
        ManIn = hardwareMap.dcMotor.get("ManIn");
        ManLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ManLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop = false;
        waitForStart();
        Stopper.setPosition(0);
        bringDownShooter(.1, 800);
        sleep(1000);
        useStopper();
        shoot(1, 400);
        ShooterB.setPower(0);
        ShooterF.setPower(0);
    }
}