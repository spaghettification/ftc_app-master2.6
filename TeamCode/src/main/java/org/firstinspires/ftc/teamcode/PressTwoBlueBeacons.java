package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "2 Blue Beacons", group = "Velocity Vortex Autonomous")
public class PressTwoBlueBeacons extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        PrepareAutonomous();
        FarBeaconPathandShootBall("blue");
        AutonomousButtonPress("blue");
        DriveSecondBeaconToFristBeacon("blue");
        AutonomousButtonPress("blue");
        requestOpModeStop();
    }
}
