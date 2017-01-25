package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;
import android.widget.Button;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/2/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "6994 Bot")
public class TeleOp extends HardwareMap{
    double LauncherCatch  = .36;
    double LauncherRelease = 1;
    @Override
    public void init() {
        HardwareMap();
        Gyro.calibrate();
        LinearproportionalConstant = 0;
        LinearintegralConstant= 0;
        LinearderivitiveConstant= 0;
    }


    @Override
    public void loop() {

        if (gamepad1.left_bumper&&gamepad1.right_bumper){
            if (!ButtonPusherMax.isPressed()){ButtonPusherActuator.setPower(1);
                if (gamepad1.x){
                ButtonPusher.setPosition(1);}
                else if (gamepad1.b){
                    ButtonPusher.setPosition(0);
                }
            }
        } else {
            ButtonPusher.setPosition(.36);
            if (!ButtonPusherMin.isPressed()){
                ButtonPusherActuator.setPower(-1);
        }
        }

        if(gamepad1.right_bumper){
            CapBallForkHolder.setPosition(.65);}
        else
            CapBallForkHolder.setPosition(0);

        if(gamepad2.right_bumper){
            BallControl.setPosition(.55);}
        else{BallControl.setPosition(.2);}

        BallCollection.setPower(-scaleInput(gamepad2.right_stick_y));
        float LeftPower = gamepad1.right_stick_y;
        float RightPower = gamepad1.left_stick_y;
        if (gamepad2.dpad_up){
            CapBallLiftLeft.setPower(1);
            CapBallLiftRight.setPower(1);
        }
        else if (gamepad2.dpad_down){
            CapBallLiftLeft.setPower(-1);
            CapBallLiftRight.setPower(-1);
        }
        else {
            CapBallLiftLeft.setPower(0);
            CapBallLiftRight.setPower(0);
            }
        if (!gamepad2.a){

            FrontLeft.setPower(Range.clip(scaleInput(LeftPower),-1,1));
            FrontRight.setPower(Range.clip(scaleInput(RightPower),-1,1));
            BackLeft.setPower(Range.clip(scaleInput(LeftPower),-1,1));
            BackRight.setPower(Range.clip(scaleInput(RightPower),-1,1));
            if (gamepad1.left_bumper&&!gamepad1.right_bumper){

                FrontLeft.setMaxSpeed(800);
                FrontRight.setMaxSpeed(800);
                BackLeft.setMaxSpeed(800);
                BackRight.setMaxSpeed(800);}
            else{
                FrontLeft.setMaxSpeed(4000);
                FrontRight.setMaxSpeed(4000);
                BackLeft.setMaxSpeed(4000);
                BackRight.setMaxSpeed(4000);}}
        if (LauncherStop.isPressed()&&gamepad2.a){
            LauncherStopServo.setPosition(LauncherRelease);
            if (Math.abs(LauncherStopServo.getPosition()-LauncherRelease)<.05){
                Launcher.setPower(1);
            }
        else if (!gamepad1.a) {LauncherStopServo.setPosition(LauncherCatch);}
    }
        else if (LauncherStop.isPressed()){
            Launcher.setPower(0);
        }
        else if (!LauncherStop.isPressed()){
            Launcher.setPower(1);
        }/*
        LauncherStopServo.setPosition(gamepad2.left_stick_x);
        telemetry.addData("LauncherServo",LauncherStopServo.getPosition());
        telemetry.addData("CapBallFork",CapBallForkHolder.getPosition());
        telemetry.addData("Ball Control",BallControl.getPosition());*/
    }

    public void stop() {
        setPower(0, 0, 0, 0);
    }

    enum Color{Red,Blue,Null}

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
        if (BackRight != null) {
            BackLeft.setPower(0);
        }
        if (BackRight != null) {
            BackRight.setPower(0);
        }
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


        Gyro = hardwareMap.gyroSensor.get(gyroSensor);

    }

    public void InitializeServoPositions() {
        if (BallControl != null) {
            BallControl.setPosition(ballControlStartPosition);
        }
        if (ButtonPusherArm != null) {
            ButtonPusherArm.setPosition(.5 );
        }

    }

    public void setPower(double FL, double FR, double BL, double BR, double Constant) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR*Constant);
        BackLeft.setPower(BL);
        BackRight.setPower(BR*Constant);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR);/*
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

    public double scaleInput(double dVal) {
        //first few numbers are zero to account for offest of joystick keeping motors at zero power until acted upon by driver
        double[] scaleArray = {0.0, 0.0, 0.2, 0.22, 0.28, 0.35, 0.40,
                0.45, 0.50, 0.55, 0.60, 0.65, 0.72, 0.85, .90, 1.0, 1.0
        };
        int index = (int) (dVal * 15.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 15) {
            index = 15;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        if (dVal < -1) {
            dVal = -1;
        }

        if (dVal > 1) {
            dVal = 1;
        }


        return dScale;
    }



}
