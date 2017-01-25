package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "1 Blue Beacon", group = "Velocity Vortex Autonomous")
public class PressOneBlueBeacon extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        PrepareAutonomous();
        FarBeaconPathandShootBall("blue");
        sleep(1000);
       AutonomousButtonPress("blue");
    }
}
