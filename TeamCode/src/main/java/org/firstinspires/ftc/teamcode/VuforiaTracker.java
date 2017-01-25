package org.firstinspires.ftc.teamcode;

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
 * Created by Trevor on 1/22/2017.
 */
public abstract class VuforiaTracker extends LinearOpMode
{
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables beacons;
    OpenGLMatrix pose;
    VuforiaTrackable Beac;
    public void InitializeVuforia(){
        parameters.cameraDirection= VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);
/*
        beacons = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        waitForStart();
        beacons.activate();*/
    }
    enum Targets{
        Wheels, Tools, Legos, Gears
    }
    public double returnTargetXTranslation(Targets target){
        double X = 0;
        switch (target){
            case Wheels:{Beac  = beacons.get(0);}break;
            case Tools: {Beac  = beacons.get(1);}break;
            case Legos: {Beac  = beacons.get(2);}break;
            case Gears: {Beac  = beacons.get(3);}
            pose = ((VuforiaTrackableDefaultListener) Beac.getListener()).getPose();
                if (pose!=null){
                    VectorF translation = pose.getTranslation();
                    X= translation.get(0);
                }
        }return X;



    }public double returnTargetYTranslation(Targets target){
        double Y = 0;
        switch (target){
            case Wheels:{Beac  = beacons.get(0);}break;
            case Tools: {Beac  = beacons.get(1);}break;
            case Legos: {Beac  = beacons.get(2);}break;
            case Gears: {Beac  = beacons.get(3);}
            pose = ((VuforiaTrackableDefaultListener) Beac.getListener()).getPose();
                if (pose!=null){
                    VectorF translation = pose.getTranslation();
                    Y= translation.get(1);
                }
        }return Y;



    }public double returnTargetTheta(Targets target){
        double Theta = 0;
        switch (target){
            case Wheels:{Beac  = beacons.get(0);}break;
            case Tools: {Beac  = beacons.get(1);}break;
            case Legos: {Beac  = beacons.get(2);}break;
            case Gears: {Beac  = beacons.get(3);}
            pose = ((VuforiaTrackableDefaultListener) Beac.getListener()).getPose();
                if (pose!=null){
                    Theta= Math.toDegrees(Math.atan2(returnTargetXTranslation(target),returnTargetYTranslation(target)));
                }
        }return Theta;



    }

}
