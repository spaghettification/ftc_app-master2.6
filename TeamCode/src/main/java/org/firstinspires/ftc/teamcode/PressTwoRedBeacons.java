package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "2 Red Beacon", group = "Velocity Vortex Autonomous")
public class PressTwoRedBeacons extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        PrepareAutonomous();
        DriveToFirstBeacon("red");
        AutonomousButtonPress("red");
        DriveFromFirstBeaconToSecondBeacon("red");
        AutonomousButtonPress("red");
    }
}
