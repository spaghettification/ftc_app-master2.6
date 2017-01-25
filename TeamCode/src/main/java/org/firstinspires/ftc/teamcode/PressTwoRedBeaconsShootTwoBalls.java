package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Trevor on 12/15/2016.
 */
@Autonomous(name = "2 Red Beacons Shoot 2", group = "Velocity Vortex Autonomous")

public class PressTwoRedBeaconsShootTwoBalls extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        PrepareAutonomous();

        DriveToFirstBeacon("red");
        AutonomousButtonPress("red");
        DriveFromFirstBeaconToSecondBeacon("red");
        AutonomousButtonPress("red");
        DriveFromSecondButtonToShootSpot("red");
        AutonomousShoot2Balls();
    }



}
