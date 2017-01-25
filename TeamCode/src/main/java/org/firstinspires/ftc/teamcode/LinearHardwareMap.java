package org.firstinspires.ftc.teamcode;

import android.graphics.Path;
import android.view.inputmethod.CorrectionInfo;
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

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByGyro_Linear;
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
import java.text.BreakIterator;
import java.util.Set;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class LinearHardwareMap extends LinearOpMode {
    double ProportionalConstant                                 =   .015;
    public double Reflectance                                   =   .3;
    double TurningConstant                                      =   .0125;

    public DcMotor                                              FrontLeft;
    public DcMotor                                              FrontRight;/*
    public DcMotor                                              BackLeft;
    public DcMotor                                              BackRight;*/
    public DcMotor                                              Catapult;
    public DcMotor                                              BallCollection;
    public DcMotor                                              CapBallLiftLeft;
    public DcMotor                                              ButtonPusherActuator;
    public GyroSensor                                           Gyro;
    public ColorSensor                                          BeaconColorSensor;
    public OpticalDistanceSensor                                LeftWhiteLineFinder;
    public OpticalDistanceSensor                                RightWhiteLineFinder;
    public ModernRoboticsI2cRangeSensor                         BlueWallFinder;
    public ModernRoboticsI2cRangeSensor                         RedWallFinder;
    public TouchSensor                                          CatapultStop;
    public TouchSensor                                          ButtonPusherMax;
    public TouchSensor                                          ButtonPusherMin;
    public Servo                                                ButtonPusherArm;
    public Servo                                                CapBallForkHolder;
    public Servo                                                WallFinderServo;
    public Servo                                                servo7;
    public Servo                                                servo8;
    public Servo                                                servo9;
    public Servo                                                servo10;
    public Servo                                                servo11;
    public Servo                                                servo12;
    public Servo                                                BallControl;
    public DeviceInterfaceModule                                DIM;


    public String frontLeftMotor                                =      "fl"; //VTOJ Port 1
    public String frontRightMotor                               =      "fr"; // VTOJ Port 2
    /*public String backLeftMotor                                 =      "bl"; //VTOL Port 1*/
    public String capBallLiftRight                                =      "cblr";//VTOL Port 2
    public String catapult                                      =      "c"; // VTAV Port 1
    public String ballCollection                                =      "bc"; //VTAV Port 2
    public String capBallLiftLeft                               =      "cbll"; //SXSX
    public String buttonPusherActuator                          =      "BPA";   //SXSX
    public String gyroSensor                                    =      "gyro";
    public String beaconColorSensor                             =      "bcs";
    public String LeftwhiteLineFinder                           =      "Lwlf";
    public String RightwhiteLineFinder                          =      "Rwlf";
    public String blueWallFinder                                =      "bluewallfinder";
    public String redWallFinder                                 =      "redwallfinder";
    public String catapultStop                                  =      "cs";
    public String capBallForkHolder                             =      "cbfh";
    public String capBallarm1                                   =      "cba1";
    public String capBallarm2                                   =      "cba2";
    public String ballControll                                  =      "ballco";
    public String buttonPusher                                  =      "bp";
    public String buttonPusherArm                               =      "bpa";
    public String buttonPusherMax                               =      "bpm";
    public String buttonPusherMin                               =      "bpmin";
    public String wallFinderServo                               =      "wallfinder";
    public double ballControlStartPosition                      =      0;
    public double ballControlEngagedPosition                    =       1;
    public double buttonPusherLeft                              =       0;
    public double buttonPusherRight                             =      .9;

    public double LinearproportionalConstant                     =       .015;
    public float LinearintegralConstant                         =       0;
    public float LinearderivitiveConstant                       =       0;

    public double CountsPerInch                                 =       89;
    public double CountsPerDegree                               =       338/19;
    public double LinearMaxCorrection                            =       .5;
    public double LinearMinCorrection                            =       .01;

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
        }
        /*if (BackRight != null) {
            BackLeft.setPower(0);
        }
        if (BackRight != null) {
            BackRight.setPower(0);
        }*/
        if (Catapult != null) {
            Catapult.setPower(0);
        }
        if (BallCollection != null) {
            BallCollection.setPower(0);
        }
        if (ButtonPusherActuator != null) {
            ButtonPusherActuator.setPower(0);
        }
        if (CapBallLiftLeft != null) {
            CapBallLiftLeft.setPower(0);
        }
    }
    public void PrepareAutonomous(){
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Gyro.calibrate();
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(Gyro.isCalibrating()){
telemetry.addData("gyro is calibrating","");
            telemetry.update();
        }telemetry.clear();
    }
    public void FarBeaconPathandShootBall(String Color){

        Drive(.3,12,0,5,false);
        while (CatapultStop.isPressed()){Catapult.setPower(1);}
            Catapult.setPower(0);
        Turn(0,.2,35,false);
        Drive(1,83,35,5,false);
        Turn(.2,0,0,false);
        sleep(500);
        FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,-.5,Reflectance);
        Drive(.2,-4,0,5,false);


    }
    DeviceInterfaceModule deviceInterfaceModule;



    public void DriveSecondBeaconToFristBeacon(String Color){
        Drive(1,-48,0,5,false);
    }

    public void DriveToFirstBeacon(String Color){
        if (Color.toLowerCase()=="blue"){
        DriveWithEncoders(.8,42,getIntegratedZValue());
        DrivetoWhiteLine("blue",.5,8,10);}
        else{
            Turn(-.25,.25,-25,true);
            DriveWithPID(.375,-30,getIntegratedZValue(),0);
            DrivetoDistance(-.25,-25,8);
            Turn(.25,-.25,0,true);
            FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,-.125,Reflectance);}
    }

    public void DriveFromFirstBeaconToSecondBeacon(String Color){
        if (Color.toLowerCase()=="blue"){
        DriveWithPID(.375,40, (int) AdaptiveWallFinderServo(),8);
        FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,.125,Reflectance);}
        else{
            DriveWithPID(.375,-40,(int) AdaptiveWallFinderServo(),8);
            FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,-.125,Reflectance);}
    }
    public void DriveFromSecondButtonToShootSpot(String Color){
        if (Color.toLowerCase()=="blue"){
        Turn(0,.25,-150,true);
        DriveWithPID(.375,24,getIntegratedZValue(),-150);}
        else{Turn(.25,0,150,true);
            DriveWithPID(.375,24,getIntegratedZValue(),150);}
    }

    public void DriveFromShootSpotToCapBallStand(String Color){
        if (Color.toLowerCase()=="blue"){
        DriveWithPID(.375,30,getIntegratedZValue(),25);
        }
        else{DriveWithPID(.375,-30,getIntegratedZValue(),-25);}
    }
    public void FindWhiteLine(OpticalDistanceSensor LeftODS,OpticalDistanceSensor RightODS,double Power,double Reflectance) {
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double LeftPower;
        double RightPower;
        while (opModeIsActive()&&LeftODS.getLightDetected()<Reflectance&&RightODS.getLightDetected()<Reflectance) {

                if (LeftODS.getLightDetected()<Reflectance){LeftPower=Power;}else{LeftPower=0;}
                if (RightODS.getLightDetected()<Reflectance){RightPower=Power;}else{RightPower=0;}
                setPower(-LeftPower, -RightPower);

        }
        Gyro.resetZAxisIntegrator();
        setPower(0,0);
        sleep(5000);
        runtime.reset();
        while (opModeIsActive()&&!isStopRequested()&&LeftODS.getLightDetected()<Reflectance&&RightODS.getLightDetected()<Reflectance) {
            if(runtime.seconds()>3){
                if (LeftODS.getLightDetected()<Reflectance){LeftPower=Power;}else{LeftPower=0;}
                if (RightODS.getLightDetected()<Reflectance){RightPower=Power;}else{RightPower=0;}
                setPower(LeftPower, RightPower);}
            else{
                if (LeftODS.getLightDetected()<Reflectance){LeftPower=Power;}else{LeftPower=0;}
                if (RightODS.getLightDetected()<Reflectance){RightPower=Power;}else{RightPower=0;}
                setPower(-LeftPower,-RightPower);
            }
        }
        sleep(500);

        runtime.reset();
    }



    public void FollowWallToWhiteLine(double Power, int TargetDistanceFromWall, double kP){
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AdaptiveWallFinderServo(0);
        runtime.reset();
        while(RightWhiteLineFinder.getLightDetected()<Reflectance||LeftWhiteLineFinder.getLightDetected()<Reflectance&&opModeIsActive()){
            FrontLeft.setPower(Power-(((BlueWallFinder.getDistance(DistanceUnit.INCH)-TargetDistanceFromWall)/100)*kP));
            FrontRight.setPower(Power+(((BlueWallFinder.getDistance(DistanceUnit.INCH)-TargetDistanceFromWall)/100)*kP));
        }setPower(0,0);
        while (RightWhiteLineFinder.getLightDetected()< Reflectance && opModeIsActive()){
            if (runtime.seconds()>2){
                setPower(0,.25);
            }
            else{
                setPower(0,-.25);
            }runtime.reset();

        }setPower(0,0);
        while (LeftWhiteLineFinder.getLightDetected()< Reflectance&& opModeIsActive()){
            if (runtime.seconds()>2){
                setPower(.25,0);
            }
            else{
                setPower(-.25,0);
            }

        }setPower(0,0);
        if (LeftWhiteLineFinder.getLightDetected()>Reflectance&&RightWhiteLineFinder.getLightDetected()>Reflectance){
            Gyro.resetZAxisIntegrator();
        }


    }

    public void DrivetoDistance(double minPower, int TargetAngle, int Distance){
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(minPower,minPower);
        FrontLeft.setTargetPosition(10000000);
        FrontRight.setTargetPosition(10000000);

        while (AdaptiveWallFinderServo()>Distance && opModeIsActive()){
            double Adjustment = 0;

            if (getIntegratedZValue()<TargetAngle){
                setPower(minPower-Adjustment,-minPower+Adjustment);
            }
            else if (getIntegratedZValue()>TargetAngle){
                setPower(-minPower+Adjustment,minPower-Adjustment);
            }
            telemetry.addData("Adjustment", Adjustment);
            telemetry.addData("Heading", getIntegratedZValue());
            telemetry.addData("Distance From Wall", BlueWallFinder.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Power", FrontLeft.getPower());
            telemetry.addData("Right Power", FrontRight.getPower());
            telemetry.update();
        }setPower(0,0);
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition());
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition());
        telemetry.clear();
    }
    enum WhiteLineFinderStatus{LeftODSHasSeenLineButNotRight, RightODSHasSeenLineButNotLeft, NoODSHasSeenLine,WhiteLineNotReached}
    WhiteLineFinderStatus whiteLineFinderStatus;

    public void DrivetoWhiteLine(String Color, double Power, int MinDistance, int MaxDistance){
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        whiteLineFinderStatus = WhiteLineFinderStatus.WhiteLineNotReached;
        double TurnSpeed = .175;
        if (Color.toLowerCase()=="blue"){
            AdaptiveWallFinderServo(45);
            while ((BlueWallFinder.getDistance(DistanceUnit.INCH)>MaxDistance&&opModeIsActive())/*||(RightWhiteLineFinder.getLightDetected()>Reflectance&&opModeIsActive())*/){
//            if (AdaptiveWallFinderServo(45)<MinDistance){
//                setPower(-Power,-Power*.6);
//                }
            if (AdaptiveWallFinderServo(45)>MaxDistance) {
                setPower(Power, Power*.6);
                }
            if (LeftWhiteLineFinder.getLightDetected()>Reflectance){
                whiteLineFinderStatus=WhiteLineFinderStatus.LeftODSHasSeenLineButNotRight;
            }
                /**
                 * The Robot will Drive until the distance sensor is happy, then the right motor will drive backwards until the gyro reads -45 degrees while the left motor remains at zero. then the robot will BACK UP to find the white line.
                 * */
            }
            if (RightWhiteLineFinder.getLightDetected()>Reflectance){
                whiteLineFinderStatus=WhiteLineFinderStatus.RightODSHasSeenLineButNotLeft;
            }/**
             * The Right motor will stop while the left motor drives forward until the left ODS sees the White line.
             */
            if (RightWhiteLineFinder.getLightDetected()<Reflectance&&whiteLineFinderStatus!=WhiteLineFinderStatus.LeftODSHasSeenLineButNotRight){
                whiteLineFinderStatus=WhiteLineFinderStatus.NoODSHasSeenLine;
            }/**
             *The robot will turn to -45 with the left motor while the right motor stays at zero. then it will drive forward until it sees the white line
             */
            telemetry.addData("Status",whiteLineFinderStatus);
            telemetry.update();
        switch (whiteLineFinderStatus){
            case LeftODSHasSeenLineButNotRight:{
                Turn(0,-.15,-42,true);
                while(LeftWhiteLineFinder.getLightDetected()<Reflectance&&RightWhiteLineFinder.getLightDetected()<Reflectance){
                    if(LeftWhiteLineFinder.getLightDetected()<Reflectance){FrontLeft.setPower(-TurnSpeed);}
                    else {FrontLeft.setPower(0);}
                    if (RightWhiteLineFinder.getLightDetected()<Reflectance){FrontRight.setPower(-TurnSpeed);}
                    else {FrontRight.setPower(0);}
                    }Gyro.resetZAxisIntegrator();
                setPower(-TurnSpeed,-TurnSpeed*.6);
                setPower(0,0);
            }break;
            case RightODSHasSeenLineButNotLeft:{
                while(LeftWhiteLineFinder.getLightDetected()<Reflectance){
                    FrontRight.setPower(0);
                    if(LeftWhiteLineFinder.getLightDetected()<Reflectance){FrontLeft.setPower(TurnSpeed);}
                    else {FrontLeft.setPower(0);}
                }Gyro.resetZAxisIntegrator();
                setPower(-TurnSpeed,-TurnSpeed*.6);
                sleep(1500);
                setPower(0,0);
            }break;
            case NoODSHasSeenLine:{
                Turn(TurnSpeed,0,-42,true);
                while(LeftWhiteLineFinder.getLightDetected()<Reflectance&&RightWhiteLineFinder.getLightDetected()<Reflectance){
                    if(LeftWhiteLineFinder.getLightDetected()<Reflectance){FrontLeft.setPower(TurnSpeed);}
                    else {FrontLeft.setPower(0);}
                    if (RightWhiteLineFinder.getLightDetected()<Reflectance){FrontRight.setPower(TurnSpeed*.6);}
                    else {FrontRight.setPower(0);}
                }Gyro.resetZAxisIntegrator();
                setPower(-TurnSpeed,-TurnSpeed*.6);
                sleep(1500);
                setPower(0,0);
            }break;
        }
        }
        if (Color.toLowerCase()=="red"){
            AdaptiveWallFinderServo(-42);
            while ((BlueWallFinder.getDistance(DistanceUnit.INCH)<MinDistance&&BlueWallFinder.getDistance(DistanceUnit.INCH)>MaxDistance&&opModeIsActive())||(LeftWhiteLineFinder.getLightDetected()>Reflectance&&opModeIsActive())){
                if (AdaptiveWallFinderServo()<MinDistance){
                    setPower(Power,Power*.6);
                }
                if (AdaptiveWallFinderServo()>MaxDistance) {
                    setPower(-Power, -Power*.6);
                }if (RightWhiteLineFinder.getLightDetected()>Reflectance){
                    whiteLineFinderStatus=WhiteLineFinderStatus.RightODSHasSeenLineButNotLeft;
                }
                /**
                 * The Robot will Drive until the distance sensor is happy, then the right motor will drive backwards until the gyro reads -45 degrees while the left motor remains at zero. then the robot will BACK UP to find the white line.
                 * */
            }
            if (LeftWhiteLineFinder.getLightDetected()>Reflectance){
                whiteLineFinderStatus=WhiteLineFinderStatus.LeftODSHasSeenLineButNotRight;
            }/**
             * The Right motor will stop while the left motor drives forward until the left ODS sees the White line.
             */
            if (LeftWhiteLineFinder.getLightDetected()<Reflectance&&whiteLineFinderStatus!=WhiteLineFinderStatus.RightODSHasSeenLineButNotLeft){
                whiteLineFinderStatus=WhiteLineFinderStatus.NoODSHasSeenLine;
            }/**
             *The robot will turn to -45 with the left motor while the right motor stays at zero. then it will drive forward until it sees the white line
             */
            telemetry.addData("Status",whiteLineFinderStatus);
            telemetry.update();

            switch (whiteLineFinderStatus){
                case RightODSHasSeenLineButNotLeft:{
                    Turn(TurnSpeed,0,42,true);
                    while(RightWhiteLineFinder.getLightDetected()<Reflectance&&LeftWhiteLineFinder.getLightDetected()<Reflectance){
                        if(RightWhiteLineFinder.getLightDetected()<Reflectance){FrontRight.setPower(TurnSpeed);}
                        else {FrontRight.setPower(0);}
                        if (LeftWhiteLineFinder.getLightDetected()<Reflectance){FrontLeft.setPower(TurnSpeed);}
                        else {FrontLeft.setPower(0);}
                    }Gyro.resetZAxisIntegrator();
                    setPower(TurnSpeed,TurnSpeed*.6);
                    sleep(1500);
                    setPower(0,0);
                }break;
                case LeftODSHasSeenLineButNotRight:{
                    while(RightWhiteLineFinder.getLightDetected()<Reflectance){
                        FrontLeft.setPower(0);
                        if(RightWhiteLineFinder.getLightDetected()<Reflectance){FrontRight.setPower(-TurnSpeed);}
                        else {FrontRight.setPower(0);}
                    }Gyro.resetZAxisIntegrator();
                    setPower(TurnSpeed,TurnSpeed*.6);
                    sleep(1500);
                    setPower(0,0);
                }break;
                case NoODSHasSeenLine:{
                    Turn(0,-TurnSpeed,42,true);
                    while(RightWhiteLineFinder.getLightDetected()<Reflectance&&LeftWhiteLineFinder.getLightDetected()<Reflectance){
                        if(RightWhiteLineFinder.getLightDetected()<Reflectance){FrontRight.setPower(-TurnSpeed*.6);}
                        else {FrontRight.setPower(0);}
                        if (LeftWhiteLineFinder.getLightDetected()<Reflectance){FrontLeft.setPower(-TurnSpeed);}
                        else {FrontLeft.setPower(0);}
                    }Gyro.resetZAxisIntegrator();
                    setPower(TurnSpeed,TurnSpeed*.6);
                    sleep(1500);
                    setPower(0,0);
                }break;
            }
            }
    }
    public void AutonomousButtonPress(String Color) {

        while (!ButtonPusherMax.isPressed()&&opModeIsActive()) {
            BeaconColorSensor.enableLed(false);
            telemetry.addData("min",ButtonPusherMin.isPressed());
            telemetry.addData("max",ButtonPusherMax.isPressed());
            telemetry.addData("blue", BeaconColorSensor.blue());
            telemetry.addData("red", BeaconColorSensor.red());
            telemetry.addData("buttonPusher", ButtonPusherArm.getPosition());
            telemetry.update();
            ButtonPusherActuator.setPower(.07);
            if (Color.toLowerCase() == "blue") {
                if (BeaconColorSensor.blue() > BeaconColorSensor.red()) {
                    ButtonPusherArm.setPosition(buttonPusherRight);
                    ButtonPusherActuator.setPower(.2);
                }
                else if (BeaconColorSensor.blue() < BeaconColorSensor.red()) {
                    ButtonPusherArm.setPosition(buttonPusherLeft);
                    ButtonPusherActuator.setPower(.2);
                }
                else {
                    telemetry.addData("Could Not See Beacon", "");
                    ButtonPusherActuator.setPower(0);
                    telemetry.update();
                    sleep(5000);
                    requestOpModeStop();
                }
            }
            if (Color.toLowerCase() == "red") {
                if (BeaconColorSensor.blue() > BeaconColorSensor.red()) {
                    ButtonPusherArm.setPosition(buttonPusherLeft);

                    ButtonPusherActuator.setPower(.2);
                }
                else if (BeaconColorSensor.blue() < BeaconColorSensor.red()) {
                    ButtonPusherArm.setPosition(buttonPusherRight);
                    ButtonPusherActuator.setPower(.2);
                }
                else {
                    telemetry.addData("Could Not See Beacon", "");
                    telemetry.update();
                    ButtonPusherActuator.setPower(0);
                    sleep(5000);
                    requestOpModeStop();
                }
            }}
        sleep(300);
        ButtonPusherActuator.setPower(0);
        while (!ButtonPusherMin.isPressed()&&opModeIsActive()) {
            ButtonPusherActuator.setPower(-.5);
        }
        ButtonPusherActuator.setPower(0);

        telemetry.update();
    }

    public void AutonomousBallShoot(){
        while (!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
        while (CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
        while (!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
    }
    public void AutonomousShoot2Balls(){ 
        if (opModeIsActive()){
            BallControl.setPosition(0);
            AutonomousBallShoot();
            BallCollection.setPower(1);
            BallControl.setPosition(1);
            sleep(2000);
            BallControl.setPosition(0);
            BallCollection.setPower(0);
            AutonomousBallShoot();
            Catapult.setPower(0);

        }}

    public double PidPowerAdjustment(int Current, int Target, double kP,double kL, double kD) {

        double LinearCumulativeerror = 0;
        double LinearproportionalCorrection;
        double LinearintegralCorrection;
        double LinearSlopeofderivitive;
        double LinearMaxCorrection = .5;
        double LinearMinCorrection =.01;        double Linearerror = Math.abs(Target - Current);
        LinearproportionalCorrection = (LinearproportionalConstant * Linearerror);
        LinearCumulativeerror += Linearerror;
        LinearintegralCorrection = (LinearintegralConstant * LinearCumulativeerror);
        LinearSlopeofderivitive = Linearerror - Linearlasterror;
        double Linearderivitivecorrection = (LinearSlopeofderivitive * LinearderivitiveConstant);


        double LinearCorrection = LinearproportionalCorrection + LinearintegralCorrection + Linearderivitivecorrection;


        return Range.clip(LinearCorrection,LinearMinCorrection,LinearMaxCorrection);

    }

    public void AutonomousHardwareMap() {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);/*
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);*/

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Catapult = hardwareMap.dcMotor.get(catapult);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        CapBallLiftLeft = hardwareMap.dcMotor.get(capBallLiftLeft);
        CapBallLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ButtonPusherActuator = hardwareMap.dcMotor.get(buttonPusherActuator);
        ButtonPusherActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CatapultStop = hardwareMap.touchSensor.get(catapultStop);
        ButtonPusherMax = hardwareMap.touchSensor.get(buttonPusherMax);
        ButtonPusherMin = hardwareMap.touchSensor.get(buttonPusherMin);

        BallControl = hardwareMap.servo.get(ballControll);
        ButtonPusherArm = hardwareMap.servo.get(buttonPusherArm);
        WallFinderServo = hardwareMap.servo.get(wallFinderServo);
        CapBallForkHolder = hardwareMap.servo.get(capBallForkHolder);
        CapBallForkHolder.setPosition(1);


        BlueWallFinder = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, blueWallFinder);

        Gyro = hardwareMap.gyroSensor.get(gyroSensor);

        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        BeaconColorSensor.enableLed(false);

        LeftWhiteLineFinder = hardwareMap.opticalDistanceSensor.get(LeftwhiteLineFinder);
        RightWhiteLineFinder=hardwareMap.opticalDistanceSensor.get(RightwhiteLineFinder);
        WallFinderServo = hardwareMap.servo.get(wallFinderServo);


    }

    public void InitializeServoPositions() {
        if (BallControl != null) {
            BallControl.setPosition(ballControlStartPosition);
        }
        if (ButtonPusherArm != null) {
            ButtonPusherArm.setPosition(.5 );
        }
        if (ButtonPusherArm != null) {
            ButtonPusherArm.setPosition(0);
        }

        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);
        CapBallForkHolder.setPosition(1);

    }

    public void setPower(double FL, double FR,/* double BL, double BR, */double Constant) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR*Constant);/*
        BackLeft.setPower(BL);
        BackRight.setPower(BR*Constant);*/
    }

    public void setPower(double FL, double FR/*, double BL, double BR*/) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR*.6);/*
        BackLeft.setPower(BL);
        BackRight.setPower(BR);*/
    }

    public void SetMode(DcMotor.RunMode mode) {
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);/*
        BackLeft.setMode(mode);
        BackRight.setMode(mode);*/
    }

    public void setMaxSpeed(int TicksPerSecond) {
        FrontLeft.setMaxSpeed(TicksPerSecond);
        FrontRight.setMaxSpeed(TicksPerSecond);/*
        BackLeft.setMaxSpeed(TicksPerSecond);
        BackRight.setMaxSpeed(TicksPerSecond);*/
    }

    public void setMaxSpeed(int TicksPerSecond, double LeftConstant, double RightConstant) {
        FrontLeft.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*LeftConstant)));
        FrontRight.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*RightConstant)));/*
        BackLeft.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*LeftConstant)));
        BackRight.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*RightConstant)));*/
    }

    public void DriveWithProportionalCorrection(double minPower, double Distance, int TargetAngle, int MaxSpeed){

        double Tolerance = .5;
        double EncoderTicks=CountsPerInch*Distance;

        setPower(minPower,minPower);

        double LeftPower = minPower;
        double RightPower = minPower;

        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getCurrentPosition() + EncoderTicks));/*
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getCurrentPosition()+ EncoderTicks));*/

        setPower(LeftPower,RightPower);

        while(FrontLeft.isBusy()&&FrontRight.isBusy()/*&&BackLeft.isBusy()&&BackRight.isBusy()*/&&opModeIsActive()) {
            double Error = getError(TargetAngle);
            double Correction = Error*ProportionalConstant;

            if (TargetAngle - getIntegratedZValue() > Tolerance) {//Angled to the left of where it should be, Left Power must be increased
                LeftPower = minPower+Correction;
                RightPower  = minPower-Correction;
                setPower(LeftPower,RightPower);

            } else if (TargetAngle - getIntegratedZValue() < Tolerance) {//Angled to the right of where it should be,Right Power must be increased
                LeftPower = minPower-Correction;
                RightPower  = minPower+Correction;
                setPower(LeftPower,RightPower);

            } else {//Robot is at the angle it should be
            }
            telemetry.addData("Current Time", Totalruntime.seconds());
            telemetry.addData("Current Heading", getIntegratedZValue());
            telemetry.addData("Front Left Power",FrontLeft.getPower());
            telemetry.addData("Front Right Power",FrontRight.getPower());/*
            telemetry.addData("Back Left Power",BackLeft.getPower());
            telemetry.addData("Back Right Power",BackRight.getPower());*/
            RobotLog.i("Current Time", Totalruntime.seconds());
            RobotLog.i("Current Heading", getIntegratedZValue());
            RobotLog.i("Front Left Power",FrontLeft.getPower());
            RobotLog.i("Front Right Power",FrontRight.getPower());/*
            RobotLog.i("Back Left Power",BackLeft.getPower());
            RobotLog.i("Back Right Power",BackRight.getPower());*/
            telemetry.update();

        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - Gyro.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double AdaptiveWallFinderServo(){
        double heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;}
        WallFinderServo.setPosition(((127+(Range.clip(heading,-90,90)*128)/90)/255));
        telemetry.addData("Wall Finder Servo Position",WallFinderServo.getPosition());
        return BlueWallFinder.getDistance(DistanceUnit.INCH);
    }
    public double AdaptiveWallFinderServo(int Heading){
        double heading = Heading;
        if (heading > 180) {
            heading -= 360;}
        WallFinderServo.setPosition(((127+(Range.clip(heading,-90,90)*128)/90)/255));
        telemetry.addData("Wall Finder Servo Position",WallFinderServo.getPosition());
        return BlueWallFinder.getDistance(DistanceUnit.INCH);
    }

    public void DriveWithEncoders(double Power, double Distance,int TargetHeading){
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        double EncoderTicks=CountsPerInch*Distance;

        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getCurrentPosition() + EncoderTicks));
        setPower( Power, Power);
        while (FrontLeft.isBusy()&&FrontRight.isBusy()&&opModeIsActive()){
            setPower(Power-((getIntegratedZValue()-TargetHeading)/100),Power+((getIntegratedZValue()-TargetHeading)/100));
            idle();
            telemetry.addData("Current Heading", getIntegratedZValue());
            telemetry.addData("Percent Traveled",(FrontLeft.getCurrentPosition()+FrontRight.getCurrentPosition())/(FrontLeft.getTargetPosition()+FrontRight.getTargetPosition()));
            telemetry.addData("Left Power", FrontLeft.getPower());
            telemetry.addData("Right Power", FrontRight.getPower());
            telemetry.update();
        }
        setPower(0,0);
        telemetry.clear();

    }

    public void DriveWithPID(double minPower,double Distance, int Current, int Target){

        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        double EncoderTicks=CountsPerInch*Distance;

        setPower(minPower,minPower);

        //setMaxSpeed(MaxSpeed);

        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getCurrentPosition() + EncoderTicks));/*
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getCurrentPosition()+ EncoderTicks));*/

        setPower(minPower,minPower);

        //setMaxSpeed(MaxSpeed);

        while(FrontLeft.isBusy()&&FrontRight.isBusy()&&opModeIsActive()) {
            double Correction = PidPowerAdjustment(Current,Target,LinearproportionalConstant,LinearintegralConstant,LinearderivitiveConstant);

            if (Target - Current > Tolerance) {//Angled to the left of where it should be, Left Power must be increased

                setPower(minPower+Correction,minPower-Correction);
               // setMaxSpeed(MaxSpeed,PidPowerAdjustment(TargetAngle),-PidPowerAdjustment(TargetAngle));

            } else if (Target - Current < Tolerance) {//Angled to the right of where it should be,Right Power must be increased

                setPower(minPower-Correction,minPower+Correction);

                //setMaxSpeed(MaxSpeed,-PidPowerAdjustment(TargetAngle),+PidPowerAdjustment(TargetAngle));

            } else {//Robot is at the angle it should be
            }
            telemetry.addData("Current Time", Totalruntime.seconds());
            telemetry.addData("Current Heading", getIntegratedZValue());
            telemetry.addData("Current Adjustment", PidPowerAdjustment(Current, Target,LinearproportionalConstant,LinearintegralConstant,LinearderivitiveConstant));
            telemetry.addData("Current Proportional Constant", LinearproportionalConstant);
            telemetry.addData("Current Integral Constant", LinearintegralConstant);
            telemetry.addData("Current Derivitive Constant", LinearderivitiveConstant);
            telemetry.addData("Front Left Power",FrontLeft.getPower());
            telemetry.addData("Front Right Power",FrontRight.getPower());/*
            telemetry.addData("Back Left Power",BackLeft.getPower());
            telemetry.addData("Back Right Power",BackRight.getPower());*/
            RobotLog.i("Current Time", Totalruntime.seconds());
            RobotLog.i("Current Heading", getIntegratedZValue());
            RobotLog.i("Current Adjustment", PidPowerAdjustment(Current, Target,LinearproportionalConstant,LinearintegralConstant,LinearderivitiveConstant));
            RobotLog.i("Current Proportional Constant", LinearproportionalConstant);
            RobotLog.i("Current Integral Constant", LinearintegralConstant);
            RobotLog.i("Current Derivitive Constant", LinearderivitiveConstant);
            RobotLog.i("Front Left Power",FrontLeft.getPower());
            RobotLog.i("Front Right Power",FrontRight.getPower());/*
            RobotLog.i("Back Left Power",BackLeft.getPower());
            RobotLog.i("Back Right Power",BackRight.getPower());*/
            telemetry.update();

        }setPower(0,0);
    }

    public void Drive(double minPower, double Distance, int TargetAngle, double TimeOut, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain = getIntegratedZValue();

        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        double EncoderTicks = CountsPerInch * Distance;
        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getTargetPosition() + EncoderTicks));/*
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getTargetPosition() + EncoderTicks));*/
        //SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(minPower,minPower,.8);
        setMaxSpeed(2500);

        while ((opModeIsActive()&&!isStopRequested()&&
                FrontLeft.isBusy() &&
                FrontRight.isBusy())
                ) {

            setPower(minPower,minPower);
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.addData("FR", FrontRight.getCurrentPosition());/*
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.addData("BR", BackRight.getCurrentPosition());*/
            telemetry.update();
        }

        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition()));
        FrontRight.setTargetPosition((int) (FrontRight.getTargetPosition()));/*
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition()));
        BackRight.setTargetPosition((int) (BackRight.getTargetPosition()));*/
        setPower(0, 0);
        sleep(300);
    }

    public void Turn(double LeftPower,double RightPower, int TargetAngle, boolean Pivot){
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int Start = getIntegratedZValue();
        double leftpower = LeftPower;
        double rightpower = RightPower;
        setPower(LeftPower,RightPower);

        while(Math.abs(getIntegratedZValue()-TargetAngle)>3){
            double Adjustment = (TargetAngle-getIntegratedZValue())*1.35;
                if (LeftPower==0){setPower(0,Math.signum(Adjustment)*(-rightpower+Adjustment));}
            else if (RightPower==0){setPower((Math.signum(Adjustment)*(leftpower+Adjustment)),0);}
            setPower((Math.signum(Adjustment)*(leftpower+Adjustment)),Math.signum(Adjustment)*(-rightpower+Adjustment));
        }setPower(0,0);
    }

    public void Turn(double power, int TargetAngle, boolean Pivot,String Direction) {
        double FrontLeftTurnPower = 0;
        double FrontRightTurnPower = 0;
        double BackLeftTurnPower = 0;
        double BackRightTurnPower = 0;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
        setMaxSpeed(2000);
        double Adjustment = TurningConstant*(Math.abs(getIntegratedZValue()-TargetAngle));
        double Power = power;
        if (Pivot) {

                if (Direction == "clockwise"){

                    FrontLeftTurnPower = 0;
                    FrontRightTurnPower =-Power ;
                    BackLeftTurnPower =0;
                    BackRightTurnPower =-Power;
                    }

                if (Direction =="counterclockwise"){
                    FrontLeftTurnPower = -Power;
                    FrontRightTurnPower = 0;
                    BackLeftTurnPower = -Power;
                    BackRightTurnPower = 0;}


            }
            else{
            if (Direction == "clockwise"){
            FrontLeftTurnPower = Power;
            FrontRightTurnPower =-Power ;
            BackLeftTurnPower = Power;
            BackRightTurnPower =-Power ;}

            if (Direction =="counterclockwise"){
                FrontLeftTurnPower = -Power;
                FrontRightTurnPower = Power;
                BackLeftTurnPower = -Power;
                BackRightTurnPower = Power;}


        }setPower(FrontLeftTurnPower, FrontRightTurnPower);
            while (opModeIsActive()&&!isStopRequested()&&Math.abs(TargetAngle- getIntegratedZValue() ) > 1.5);
            {if (Pivot) {

                if (Direction == "clockwise"){

                    FrontLeftTurnPower = 0;
                    FrontRightTurnPower =-Power+Adjustment ;
                    BackLeftTurnPower =0;
                    BackRightTurnPower =-Power+Adjustment;
                }

                if (Direction =="counterclockwise"){
                    FrontLeftTurnPower = 0;
                    FrontRightTurnPower = Power-Adjustment;
                    BackLeftTurnPower = 0;
                    BackRightTurnPower = Power-Adjustment;}


            }
            else{
                if (Direction == "clockwise"){
                    FrontLeftTurnPower = Power-Adjustment;
                    FrontRightTurnPower =-Power+Adjustment ;
                    BackLeftTurnPower = Power-Adjustment;
                    BackRightTurnPower =-Power+Adjustment ;}

                if (Direction =="counterclockwise"){
                    FrontLeftTurnPower = -Power+Adjustment;
                    FrontRightTurnPower = Power-Adjustment;
                    BackLeftTurnPower = -Power+Adjustment;
                    BackRightTurnPower = Power-Adjustment;}

                setPower(FrontLeftTurnPower, FrontRightTurnPower);idle();
                telemetry.addData(">", "Turning!");
                telemetry.update();

            }
            setPower(0, 0);
    }}


    }


