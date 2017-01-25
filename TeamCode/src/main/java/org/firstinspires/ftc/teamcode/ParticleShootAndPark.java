package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Trevor on 12/15/2016.
 */

@Autonomous(name = "Shoot 2 and Park", group = "Velocity Vortex Autonomous")
public class ParticleShootAndPark extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        PrepareAutonomous();
        CapBallForkHolder.setPosition(1);
SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CapBallForkHolder.setPosition(1);
        setPower(.5,.5);

        CapBallForkHolder.setPosition(1);
        sleep(1000);


        CapBallForkHolder.setPosition(1);
        setPower(0,0);
        CapBallForkHolder.setPosition(1);
        AutonomousShoot2Balls();
        CapBallForkHolder.setPosition(1);
        setPower(1,1);
        CapBallForkHolder.setPosition(1);
        sleep(2500);
        CapBallForkHolder.setPosition(1);
        setPower(0,0);
        CapBallForkHolder.setPosition(1);
    }
}
