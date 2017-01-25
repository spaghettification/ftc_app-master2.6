package org.firstinspires.ftc.teamcode;

import android.view.animation.LinearInterpolator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Trevor on 12/18/2016.
 */

@Autonomous(name = "PID Auto Config", group = "Velocity Vortex Autonomous")
public class PID_AutoConfig extends LinearHardwareMap {
    double DifferenceBeforeLast = 0;
    double LastDifference = 0;
    double Difference = 0;

    enum DeltaConstants {Raise, Lower}

    DeltaConstants deltaConstants;

    @Override
    public void runOpMode(){
        deltaConstants = DeltaConstants.Raise;
        LinearproportionalConstant = 0;
        LinearintegralConstant = 0;
        LinearderivitiveConstant = 0;
        PrepareAutonomous();
        waitForStart();

        while (Difference > 1 && opModeIsActive()) {
            FindDifference(0);
            EditConstants(.1, 0, 0);
            Gyro.resetZAxisIntegrator();
            Turn(.175, -.175, 180, false);
            Gyro.resetZAxisIntegrator();
        }

    }

    public void FindDifference(int TargetAngle) {
        int StartAngle = getIntegratedZValue();
        DriveWithPID(.5, 24, TargetAngle, 2500);
        Difference = Math.abs(getIntegratedZValue() - StartAngle);
        RobotLog.i("Difference", Difference);
        RobotLog.i("kP", LinearproportionalConstant);
        RobotLog.i("kI", LinearintegralConstant);
        RobotLog.i("kD", LinearderivitiveConstant);
    }

    public void EditConstants(double ProportionalDelta, double IntegralDelta, double DerivitiveDelta) {
        if (Difference <= LastDifference) {
            if (deltaConstants == DeltaConstants.Raise) {
                LinearproportionalConstant += ProportionalDelta;
                LinearintegralConstant += IntegralDelta;
                LinearderivitiveConstant += DerivitiveDelta;
            } else {
                LinearproportionalConstant -= ProportionalDelta;
                LinearintegralConstant -= IntegralDelta;
                LinearderivitiveConstant -= DerivitiveDelta;
            }
        }
        if (Difference > LastDifference) {
            if (deltaConstants == DeltaConstants.Raise) {
                deltaConstants = DeltaConstants.Lower;
                LinearproportionalConstant -= ProportionalDelta;
                LinearintegralConstant -= IntegralDelta;
                LinearderivitiveConstant -= DerivitiveDelta;
            } else {
                deltaConstants = DeltaConstants.Raise;
                LinearproportionalConstant += ProportionalDelta;
                LinearintegralConstant += IntegralDelta;
                LinearderivitiveConstant += DerivitiveDelta;
            }
        }

    }
}
