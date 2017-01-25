package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.*;
import com.qualcomm.robotcore.util.Range;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Trevor on 1/14/2017.
 */
public abstract class AutonomousTemplate extends LinearOpMode implements HardwareRegister {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor CapBallLift1;
    DcMotor CapBallLift2;
    DcMotor Launcher;
    DcMotor BallCollection;

    Servo CapBallForkHolder;
    Servo BallControl;


    TouchSensor LauncherStop;

    GyroSensor Gyro;

    ModernRoboticsI2cRangeSensor WallFinder;

    TouchSensor ButtonPusherMax;
    ColorSensor ButtonPusherColorSensor;
    Servo ButtonPusher;
    CRServo ButtonPusherActuator;
    OpticalDistanceSensor FrontODS;
    OpticalDistanceSensor BackODS;


    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.03;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable
    static final double     I_Turn_Coeff = .0;
    static final double     I_Drive_Coeff = .0;
    static final double     D_Turn_Coeff  = .00;
    static final double     D_Drive_Coeff  = .00;

    double SlopeOfDerivitive;
    double CuumulativeError = 0;
    double PreviousError = 0;

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int COUNTS_PER_INCH = 89;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode Set","");
        telemetry.update();

            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = FrontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = BackLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = BackRight.getCurrentPosition() + moveCounts;
        telemetry.addData("Target Made", "");
        telemetry.update();

            // Set Target and Turn On RUN_TO_POSITION
            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);

        telemetry.addData("Target TargetSet", "");
        telemetry.addData("Front Left", FrontLeft.getTargetPosition());
        telemetry.addData("Front Right", FrontRight.getTargetPosition());
        telemetry.addData("Back Left", BackLeft.getTargetPosition());
        telemetry.addData("BackRight", BackRight.getTargetPosition());
        telemetry.addData("Front Left", FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right", FrontRight.getCurrentPosition());
        telemetry.addData("Back Left", BackLeft.getCurrentPosition());
        telemetry.addData("BackRight", BackRight.getCurrentPosition());

        telemetry.update();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
                if (!ButtonPusherMax.isPressed()){
                    ButtonPusherActuator.setPower(1);
                }
                else {
                    if(ButtonPusherColorSensor.blue()>ButtonPusherColorSensor.red()){
                        ButtonPusher.setPosition(.8);
                    }
                    else if (ButtonPusherColorSensor.blue()<ButtonPusherColorSensor.red()){
                        ButtonPusher.setPosition(.2);
                    }
                    else {ButtonPusher.setPosition(.36);}
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF,I_Drive_Coeff,D_Drive_Coeff);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                FrontLeft.setPower(Range.clip(leftSpeed,0,1));
                FrontRight.setPower(Range.clip(rightSpeed,0,1));
                BackLeft.setPower(Range.clip(leftSpeed,0,1));
                BackRight.setPower(Range.clip(rightSpeed,0,1));

                // Display drive status for the driver.
                /*
                telemetry.addData("Heading",  "%5.1f",getIntegratedZValue());
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontLeftTarget,  newFrontRightTarget,newBackLeftTarget,newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),BackLeft.getCurrentPosition(),BackRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();*/
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation =createMatrix(0,0,0,0,0,0);

    public double getVuforiaTheta(){
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        if (latestLocation != null)
            lastKnownLocation = latestLocation;
        float[] coordinates = lastKnownLocation.getTranslation().getData();
        double Theta  = -Math.toDegrees(Math.atan2(coordinates[0],coordinates[2]));
        return Theta;
    }

    double Correction;
    public double SetMotorCorrection(DcMotor motor, double MinPower){
        double Percent = (motor.getCurrentPosition()/motor.getTargetPosition());
        double Constant = 0;
        while (Percent<=.45){Correction = ((Constant*Percent)* MinPower);}
        while (Percent>.45 && Percent<.75){Correction=0;}
        while (Percent>.75){Correction = (-MinPower*(Constant*Percent));}
        return Correction;
    }

    public String frontLeftMotor                                =      "fl"; //VTOJ Port 1
    public String frontRightMotor                               =      "fr"; // VTOJ Port 2
    public String backLeftMotor                                 =      "bl"; //VTOL Port 1
    public String backRightMotor                                =      "br";//VTOL Port 2
    public String launcher                                      =      "l"; // VTAV Port 1
    public String ballCollection                                =      "bc"; //VTAV Port 2
    public String capBallLiftLeft                               =      "cbll"; //SXSX
    public String capBallLiftRight                              =      "cblr"; //SXSX
    public String buttonPusherActuator                          =      "BPA";   //SXSX
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
    public String buttonPusher                                  =      "bp";
    public String buttonPusherArm                               =      "bpa";
    public String buttonPusherMax                               =      "bpm";
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

    public Servo                                                LauncherStopServo;

    public DcMotor                                              CapBallLiftLeft;
    public DcMotor                                              CapBallLiftRight;
    public void hardwareMap(){
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Launcher = hardwareMap.dcMotor.get(launcher);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        CapBallLiftLeft = hardwareMap.dcMotor.get(capBallLiftLeft);
        CapBallLiftRight = hardwareMap.dcMotor.get(capBallLiftRight);
        CapBallLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        LauncherStop = hardwareMap.touchSensor.get(launcherStop);

        BallControl = hardwareMap.servo.get(ballControll);
        CapBallForkHolder = hardwareMap.servo.get(capBallForkHolder);
        LauncherStopServo = hardwareMap.servo.get(launcherStopServo);
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);

        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gyro.calibrate();


        ButtonPusherMax = hardwareMap.touchSensor.get(buttonPusherMax);
        ButtonPusherColorSensor = hardwareMap.colorSensor.get(buttonPusherColorSensor);
        ButtonPusher = hardwareMap.servo.get(buttonPusher);
        ButtonPusherActuator = hardwareMap.crservo.get(buttonPusherActuator);
        ButtonPusherActuator.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontODS = hardwareMap.opticalDistanceSensor.get(frontODS);
        BackODS = hardwareMap.opticalDistanceSensor.get(backODS);
        ButtonPusher.setPosition(.36);
        LauncherStopServo.setPosition(.36);
        BallControl.setPosition(.2);
        CapBallForkHolder.setPosition(0);


    }
    public void initialize(){
        hardwareMap();
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMaxSpeed(4000);
        Gyro.calibrate();
        while (Gyro.isCalibrating()){}
    }
    public void SetMode(DcMotor.RunMode mode){
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);
        BackLeft.setMode(mode);
        BackRight.setMode(mode);
    }
    public void SetMaxSpeed(int EncoderTick){
        FrontLeft.setMaxSpeed(EncoderTick);
        FrontRight.setMaxSpeed(EncoderTick);
        BackLeft.setMaxSpeed(EncoderTick);
        BackRight.setMaxSpeed(EncoderTick);
    }
    public void SetPower(double FLPower,double FRPower,double BLPower,double BRPower){
        FrontLeft.setPower(com.qualcomm.robotcore.util.Range.clip(FLPower,0,1));
        FrontRight.setPower(com.qualcomm.robotcore.util.Range.clip(FRPower,0,1));
        BackLeft.setPower(com.qualcomm.robotcore.util.Range.clip(BLPower,0,1));
        BackRight.setPower(Range.clip(BRPower,0,1));
    }
    public void setPower(double FLPower,double FRPower,double BLPower,double BRPower){
        FrontLeft.setPower(FLPower);
        FrontRight.setPower(FRPower);
        BackLeft.setPower(BLPower);
        BackRight.setPower(BRPower);
    }
    public int getIntegratedZValue() {
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }
    public void gyroTurn (  double speed, double angle) {
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF,I_Turn_Coeff,D_Turn_Coeff)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    private OpenGLMatrix phoneLocation;
    private static final String VUFORIA_KEY = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu"; // Insert your own key here


    boolean onHeading(double speed, double angle, double PCoeff,double ICoeff, double DCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff,ICoeff,DCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        FrontLeft.setPower(leftSpeed);
        FrontRight.setPower(rightSpeed);
        BackLeft.setPower(leftSpeed);
        BackRight.setPower(rightSpeed);

        // Display it for the driver.
        /*
        telemetry.addData("Current", "%5.2f", getIntegratedZValue());
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);*/

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle -Gyro.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff, double ICoeff, double DCoeff) {
        CuumulativeError+=error;
        SlopeOfDerivitive = error-PreviousError;
        double Proportional = error * PCoeff;
        double Integral = CuumulativeError * ICoeff;
        double Derivitive = SlopeOfDerivitive * DCoeff;
        PreviousError=error;
        return Range.clip(Proportional+Integral+Derivitive, -1, 1);
    }
    public double getVuforiaError(VuforiaTracker.Targets Target) {

        double robotError;
        robotError = getVuforiaTheta();
        return robotError;
    }

    public double getVuforiaSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void vuforiaDrive ( double speed,
                            double distance,double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int COUNTS_PER_INCH = 89;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode Set","");
        telemetry.update();

        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newFrontLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
        newFrontRightTarget = FrontRight.getCurrentPosition() + moveCounts;
        newBackLeftTarget = BackLeft.getCurrentPosition() + moveCounts;
        newBackRightTarget = BackRight.getCurrentPosition() + moveCounts;
        telemetry.addData("Target Made", "");
        telemetry.update();

        // Set Target and Turn On RUN_TO_POSITION
        FrontLeft.setTargetPosition(newFrontLeftTarget);
        FrontRight.setTargetPosition(newFrontRightTarget);
        BackLeft.setTargetPosition(newBackLeftTarget);
        BackRight.setTargetPosition(newBackRightTarget);

        telemetry.addData("Target TargetSet", "");
        telemetry.addData("Front Left", FrontLeft.getTargetPosition());
        telemetry.addData("Front Right", FrontRight.getTargetPosition());
        telemetry.addData("Back Left", BackLeft.getTargetPosition());
        telemetry.addData("BackRight", BackRight.getTargetPosition());
        telemetry.addData("Front Left", FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right", FrontRight.getCurrentPosition());
        telemetry.addData("Back Left", BackLeft.getCurrentPosition());
        telemetry.addData("BackRight", BackRight.getCurrentPosition());

        telemetry.update();

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (opModeIsActive() &&
                (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
            if (!ButtonPusherMax.isPressed()){
                ButtonPusherActuator.setPower(1);
            }
            else {
                if(ButtonPusherColorSensor.blue()>ButtonPusherColorSensor.red()){
                    ButtonPusher.setPosition(.8);
                }
                else if (ButtonPusherColorSensor.blue()<ButtonPusherColorSensor.red()){
                    ButtonPusher.setPosition(.2);
                }
                else {ButtonPusher.setPosition(.36);}
            }

            // adjust relative speed based on heading error.
            error = getVuforiaError(VuforiaTracker.Targets.Wheels);
            steer = getVuforiaSteer(error, P_DRIVE_COEFF*.035);


            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if any one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }
            FrontLeft.setPower(Range.clip(leftSpeed,0,1));
            FrontRight.setPower(Range.clip(rightSpeed,0,1));
            BackLeft.setPower(Range.clip(leftSpeed,0,1));
            BackRight.setPower(Range.clip(rightSpeed,0,1));

            // Display drive status for the driver.
                /*
                telemetry.addData("Heading",  "%5.1f",getIntegratedZValue());
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontLeftTarget,  newFrontRightTarget,newBackLeftTarget,newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),BackLeft.getCurrentPosition(),BackRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();*/
        }

        // Stop all motion;
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        // Turn off RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
