package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by johnsjim on 10/7/2016.
 */

@Autonomous(name = "Vuforia", group = "Autonomous")
public class vuforiaOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AcHhtA//////AAAAGeVET6zUZUmgvKl9yQJdHRaMHQjyM8gUfTCF0Ibjh3nTdVnn6jlaN35r2WYussGtezxeMvUgQ8m3rM2PUMJmL/dc1uGGpRsGPWkjXW2/2U2IFjLyea9VOMZL/uC0aU0K+4DdczxOeU31t6f1LQI3OyMPAvxowG8RNIAAK0wc3gYkssfulsGMkDb3UdNf5Z1vY4dGo7bzqEfSF28FWBo8p/royv5kBiyMDV13Yl4fQlp7At6LipSnGwEgnPP+fYN38XPggDMMeDhqfaN2KUbUUClZbbGEdQsiyGxMHOKM1S757Pl4D7WvFUjj3qhaSZnEgyE67ZF3w/3hG3eMkLh40XHqUGxzgK9Ulv2+s1EQU9N8";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.BUILDINGS;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()) {
            for(VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }
}
