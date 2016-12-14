package org.firstinspires.ftc.teamcode.Autonomous.OpModes.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.OpModes.AutoOpMode;

/**
 * Created by viperbots on 12/14/2016.
 */
@Autonomous(name ="TestTurn", group ="Autonomous")
public class TestTurn extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
    }

    public void correctionsTurnRight(double power, float angle) throws InterruptedException {
        {
            float beforeAngle = getGyroYaw();
            telemetry.addData("beforeYawAngle", beforeAngle);
            telemetry.update();
            while(Math.abs(getGyroYaw() - beforeAngle) < angle)
            {
                turnRight(power);
                double difference = Math.abs(getGyroYaw() - beforeAngle);
                while (Math.abs(getGyroYaw() - beforeAngle) > 2) {
                    FR.setPower(power * (1 + difference * correction));
                    BR.setPower(power * (1 + difference * correction));
                    FL.setPower(power);
                    BL.setPower(-power);
                    telemetry.addData("LeftPower", FR.getPower());
                    telemetry.addData("RightPower", BR.getPower());
                    telemetry.update();
                    idle();
                }
                idle();
            }
            beforeAngle = getGyroYaw();
            telemetry.addData("afterYawAngle", beforeAngle);
            telemetry.update();
        }
    }
}
