package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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

/**
 * Created by Trevor on 1/14/2017.
 */
@Autonomous(name = "2 acon", group = "Velocity Vortex Autonomous")
public class TestPath extends AutonomousTemplate {


    VuforiaLocalizer vuforia;
    VuforiaTrackables beacons;
    OpenGLMatrix pose;
    VuforiaTrackable Beac;
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation =createMatrix(0,0,0,0,0,0);

    private OpenGLMatrix phoneLocation;
     private static final String VUFORIA_KEY = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu"; // Insert your own key here

     private float robotX = 0;
    private float robotY = 0;
     private float robotAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initialize();
        setupVuforia();
        visionTargets.activate();
        waitForStart();

             gyroDrive(.5,46 ,0);
             gyroTurn(.1,88);
             vuforiaDrive(.25,40,90);
         /*sleep(1000);
         gyroDrive(.5,-14,90);
         gyroTurn(.1,0);
         gyroDrive(.5,53,0);
         gyroTurn(.1,90);
         gyroDrive(.5,40,90);*/

    }



    public double getVuforiaTheta(){
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        if (latestLocation != null)
            lastKnownLocation = latestLocation;
        float[] coordinates = lastKnownLocation.getTranslation().getData();
        double Theta  = coordinates[0]/10;
        telemetry.addData("difference",Theta);
        telemetry.update();
        return Theta;
    }


    private void setupVuforia()
    {
        // Setup parameters to create localizer
         parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
         parameters.vuforiaLicenseKey = VUFORIA_KEY;
         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
         parameters.useExtendedTracking = false;
         vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

         // These are the vision targets that we want to use
         // The string needs to be the name of the appropriate .xml file in the assets folder
         visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
         Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

         // Setup the target to be tracked
         target = visionTargets.get(0); // 0 corresponds to the wheels target
         target.setName("Wheels Target");
         target.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

         // Set phone location on robot
         phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

         // Setup listener and inform it of phone information
         listener = (VuforiaTrackableDefaultListener) target.getListener();
         listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
       }
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
        multiplied(Orientation.getRotationMatrix(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
        }
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
         }
    }

    /**
             gyroDrive(.5,44 ,0);
             gyroTurn(.1,88);
             gyroDrive(.5,40,90);
             sleep(1000);
             gyroDrive(.5,-14,90);
             gyroTurn(.1,0);
             gyroDrive(.5,53,0);
             gyroTurn(.1,90);
             gyroDrive(.5,40,90);
             This Program sometimes pushes both buttons sometimes */

            //button pusher stow at .36
            //button pusher right at 0 left at 1
            //lauincher stop catch at .36 release at 1
            //ball control catch at .2 release at .55
            //fork holder closed at 0 open at .65


            //gyroDrive(1,60,0);
        /*
        gyroTurn(.6,90);
        gyroDrive(1,50,90);
        gyroDrive(.25,-5,90);
        if (BeaconColorSensor.blue()>BeaconColorSensor.red()){
            gyroDrive(.5,8,90);
            gyroDrive(.5,-8,90);
        }
        gyroTurn(.5,0);
        gyroDrive(1,48,0);
        gyroTurn(.5,90);
        gyroDrive(1,10,90);
        gyroDrive(.5,-5,90);
        if (BeaconColorSensor.blue()>BeaconColorSensor.red()){
            sleep(5000);
            gyroDrive(.5,8,90);
            gyroDrive(.5,-8,90);
        }

        requestOpModeStop();*/
