package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Set;

/**
 * Created by Trevor on 12/24/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Range", group = "Velocity Vortex Autonomous")
public class Range extends LinearHardwareMap {

    public ModernRoboticsI2cRangeSensor BlueWallFinder;
    @Override
    public void runOpMode() throws InterruptedException {
        double DrivingConstant = 0;
        AutonomousHardwareMap();
        Gyro.calibrate();
        waitForStart();
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turn(.1,.1,20,true);
        telemetry.addData("Left Position",FrontLeft.getCurrentPosition());
        telemetry.addData("Right Position",FrontRight.getCurrentPosition());
        telemetry.addData("Heading",getIntegratedZValue());
        telemetry.addData("Left Counts per Degree", getIntegratedZValue()/FrontLeft.getCurrentPosition());
        telemetry.addData("Right Counts per Degree", getIntegratedZValue()/FrontRight.getCurrentPosition());
        telemetry.update();
    }

}
