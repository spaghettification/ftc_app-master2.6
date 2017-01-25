package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "2 Blue Beacons 2 Balls", group = "Velocity Vortex Autonomous")
public class PressTwoBlueBeaconsShootTwoBalls extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        PrepareAutonomous();

        DriveToFirstBeacon("blue");
        AutonomousButtonPress("blue");
        DriveFromFirstBeaconToSecondBeacon("blue");
        AutonomousButtonPress("blue");
        DriveFromSecondButtonToShootSpot("blue");
        AutonomousShoot2Balls();
    }



}
