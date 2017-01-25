package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Trevor on 12/19/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MaxSpeedTest", group = "6994 Bot")
public class MaxSpeedTest extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        PrepareAutonomous();
        while (opModeIsActive()){
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setPower(gamepad1.left_stick_y);
            FrontLeft.setMaxSpeed(Range.clip((int) (6000*gamepad1.right_stick_y),3000,6000));
            telemetry.addData("Power", FrontLeft.getPower());
            telemetry.addData("Speed", FrontLeft.getMaxSpeed());
            telemetry.addData("","When power alone is changed does the motor go faster?");


        }

    }
}
