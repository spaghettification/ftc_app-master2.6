package org.firstinspires.ftc.teamcode;

import android.view.animation.LinearInterpolator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.security.Guard;
import java.util.Set;

import javax.xml.parsers.FactoryConfigurationError;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "PID Manual Config", group = "Velocity Vortex Autonomous")
public class PID_Configuration extends LinearHardwareMap {
    boolean Run = false;
    int HeadingAtStart;
    int VariableSwitcher=0;
    int variableswitcher;

    @Override
    public void runOpMode(){
        LinearproportionalConstant=0;
        LinearintegralConstant=0;
        LinearderivitiveConstant=0;
        AutonomousHardwareMap();
        StopAllMotors();
        Gyro.calibrate();
            while (Gyro.isCalibrating()){

             }
             waitForStart();
            while (!gamepad1.a){
                telemetry.addData("P", LinearproportionalConstant);
                telemetry.addData("I", LinearintegralConstant);
                telemetry.addData("D", LinearderivitiveConstant);
                telemetry.addData("Press A on Gamepad 1 once you are happy with the values","");
                telemetry.update();

                double Theta=.01;

                if (gamepad1.x){Theta+=.01;}
                if (gamepad1.b) {Theta-=.01;}

                if (gamepad2.dpad_up){LinearproportionalConstant+=Theta;}
                if (gamepad2.dpad_down){LinearproportionalConstant-=Theta;}

                if (gamepad2.dpad_left){LinearintegralConstant+=Theta;}
                if (gamepad2.dpad_right){LinearintegralConstant-=Theta;}

                if (gamepad2.y){LinearderivitiveConstant+=Theta;}
                if (gamepad2.a){LinearderivitiveConstant-=Theta;}

            }

            DriveWithPID(.25,48,getIntegratedZValue(),0);
        }
    }

