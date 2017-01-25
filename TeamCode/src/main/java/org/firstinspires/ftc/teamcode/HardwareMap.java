package org.firstinspires.ftc.teamcode;

import android.graphics.Path;
import android.widget.Button;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.sql.Ref;
import java.util.Set;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class HardwareMap extends OpMode {

        double ProportionalConstant                                 =   .015;
        public double Reflectance                                   =   .6;
        double TurningConstant                                      =   .0125;

        public DcMotor                                              FrontLeft;
        public DcMotor                                              FrontRight;
        public DcMotor                                              BackLeft;
        public DcMotor                                              BackRight;
        public DcMotor                                              Launcher;
        public DcMotor                                              BallCollection;
        public DcMotor                                              CapBallLiftLeft;
        public DcMotor                                              CapBallLiftRight;
        public GyroSensor                                           Gyro;
        public ColorSensor                                          BeaconColorSensor;
        public OpticalDistanceSensor                                LeftWhiteLineFinder;
        public OpticalDistanceSensor                                RightWhiteLineFinder;
        public ModernRoboticsI2cRangeSensor                         BlueWallFinder;
        public ModernRoboticsI2cRangeSensor                         RedWallFinder;
        public TouchSensor                                          LauncherStop;
        public TouchSensor                                          ButtonPusherMin;
        public Servo                                                ButtonPusherArm;
        public Servo                                                CapBallForkHolder;
        public Servo                                                WallFinderServo;
        public Servo                                                LauncherStopServo;
        public Servo                                                servo8;
        public Servo                                                servo9;
        public Servo                                                servo10;
        public Servo                                                servo11;
        public Servo                                                servo12;
        public Servo                                                BallControl;
        public DeviceInterfaceModule                                DIM;
    String buttonPusherMax = "buttonpushermax";
    String buttonPusherColorSensor = "buttonpushercolorsensor";
    String buttonPusher = "buttonpusher";
    String buttonPusherActuator = "buttonpusheractuator";
    String frontODS = "frontods";
    String backODS = "backods";

    TouchSensor ButtonPusherMax;
    ColorSensor ButtonPusherColorSensor;
    Servo ButtonPusher;
    CRServo ButtonPusherActuator;
    OpticalDistanceSensor FrontODS;
    OpticalDistanceSensor BackODS;

        public String frontLeftMotor                                =      "fl"; //VTOJ Port 1
        public String frontRightMotor                               =      "fr"; // VTOJ Port 2
        public String backLeftMotor                                 =      "bl"; //VTOL Port 1
        public String backRightMotor                                =      "br";//VTOL Port 2
        public String launcher                                      =      "l"; // VTAV Port 1
        public String ballCollection                                =      "bc"; //VTAV Port 2
        public String capBallLiftLeft                               =      "cbll"; //SXSX
        public String capBallLiftRight                              =      "cblr"; //SXSX
        public String gyroSensor                                    =      "gyro";
        public String beaconColorSensor                             =      "bcs";
        public String LeftwhiteLineFinder                           =      "Lwlf";
        public String RightwhiteLineFinder                          =      "Rwlf";
        public String blueWallFinder                                =      "bluefinder";
        public String redWallFinder                                 =      "redfinder";
        public String launcherStop                                  =      "lsts";
        public String capBallForkHolder                             =      "cbfh";
        public String capBallarm1                                   =      "cba1";
        public String capBallarm2                                   =      "cba2";
        public String ballControll                                  =      "ballco";
        public String buttonPusherArm                               =      "bpa";
        public String buttonPusherMin                               =      "bpmin";
        public String wallFinderServo                               =      "wf";
        public String launcherStopServo                             =       "ls";
        public double ballControlStartPosition                      =      .7;
        public double ballControlEngagedPosition                    =       0;
        public double buttonPusherLeft                              =       0;
        public double buttonPusherRight                             =      .9;

        public float LinearproportionalConstant                     =       0;
        public float LinearintegralConstant                         =       0;
        public float LinearderivitiveConstant                       =       0;

        public double CountsPerInch                                 =       89;
        public float LinearMaxCorrection                            =       100;
        public float LinearMinCorrection                            =       15;

        double Tolerance = 0;


        String VuforiaLicenseKey = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu";
        float Linearlasterror=0;
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime Totalruntime = new ElapsedTime();

        public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
            int heading = Gyro.getHeading();
            if (heading > 180) {
                heading -= 360;
            }
            return heading;
        }

        public void StopAllMotors() {
            if (FrontLeft != null) {
                FrontLeft.setPower(0);
            }
            if (FrontRight != null) {
                FrontRight.setPower(0);
            }/*
            if (BackRight != null) {
                BackLeft.setPower(0);
            }
            if (BackRight != null) {
                BackRight.setPower(0);
            }*/
            if (Launcher != null) {
                Launcher.setPower(0);
            }
            if (BallCollection != null) {
                BallCollection.setPower(0);
            }
            if (CapBallLiftLeft != null) {
                CapBallLiftLeft.setPower(0);
            }
        }


        public void HardwareMap() {
            FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
            FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
            BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
            BackRight = hardwareMap.dcMotor.get(backRightMotor);

            FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

            Launcher = hardwareMap.dcMotor.get(launcher);
            BallCollection = hardwareMap.dcMotor.get(ballCollection);
            CapBallLiftLeft = hardwareMap.dcMotor.get(capBallLiftLeft);
            CapBallLiftRight = hardwareMap.dcMotor.get(capBallLiftRight);
            CapBallLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            LauncherStop = hardwareMap.touchSensor.get(launcherStop);

            BallControl = hardwareMap.servo.get(ballControll);
            CapBallForkHolder = hardwareMap.servo.get(capBallForkHolder);
            LauncherStopServo = hardwareMap.servo.get(launcherStopServo);

            ButtonPusherMax = hardwareMap.touchSensor.get(buttonPusherMax);
            ButtonPusherColorSensor = hardwareMap.colorSensor.get(buttonPusherColorSensor);
            ButtonPusher = hardwareMap.servo.get(buttonPusher);
            ButtonPusherActuator = hardwareMap.crservo.get(buttonPusherActuator);
            FrontODS = hardwareMap.opticalDistanceSensor.get(frontODS);
            BackODS = hardwareMap.opticalDistanceSensor.get(backODS);

            Gyro = hardwareMap.gyroSensor.get(gyroSensor);



        }
        public double AdaptiveWallFinderServo(){
            WallFinderServo.setPosition(Range.clip(((127-(getIntegratedZValue()*128)/90)/255),0,1));
            return BlueWallFinder.getDistance(DistanceUnit.INCH);

    }





}


