package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "1 Red Beacon", group = "Velocity Vortex Autonomous")
public class PressOneRedBeacon extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        PrepareAutonomous();
        DriveToFirstBeacon("red");
        AutonomousButtonPress("red");
    }
}
