package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Trevor on 12/31/2016.
 */
public abstract interface  HardwareRegister {
    String frontLeft = "fl";
    String frontRight = "fr";
    String backLeft = "bl";
    String backRight = "br";

    String capBallLift1 = "cbl1";
    String capBallLift2 = "cbl2";
    String launcher = "c";
    String ballCollection = "bc";

    String capBallForkHolder = "cbfh";
    String ballControl = "bco";
    String adaptiveWallFinderServo = "wfs";

    String launcherStop = "ls";
    String buttonPusherMin = "bpmin";
    String buttonPusherAtBeacon = "bpb";

    String beaconColorSensor = "bcs";

    String leftWhiteLineFinder = "lwlf";
    String rightWhiteLineFinder = "rwlf";

    String gyro = "gyro";

    String wallFinder = "wf";

    String buttonPusherMax = "buttonpushermax";
    String buttonPusherColorSensor = "buttonpushercolorsensor";
    String buttonPusher = "buttonpusher";
    String buttonPusherActuator = "buttonpusheractuator";
    String frontODS = "frontods";
    String backODS = "backods";

    double Reflectance = .6;

    int AllowBallThrough = 1;
    int DoNotAllowBallThrough = 0;

    int HoldCapBallFork = 1;
    int ReleaseCapBallFork = 0;

    int StowRangeSensor = 1;

    int EncoderCountsPerInch = 89/2;

    int WheelCircumference = (int) (4*Math.PI);

    int InchPerSecond = 30;

    int EncoderTickPerSecond = 1440*((InchPerSecond/2)/WheelCircumference);


}
