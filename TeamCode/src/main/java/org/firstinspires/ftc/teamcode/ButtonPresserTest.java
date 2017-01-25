package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Trevor on 12/21/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ButtonPusherTest", group = "6994 Bot")
public class ButtonPresserTest extends LinearHardwareMap {
    @Override
    /**ColorSensor is to the right of the servo when looking from the prespective of the touch sensor, so that is how i will lable the positoins. closest to the touch sensor will be BeaconRight at .9 and to the left will be BeaconLeft at 0*/
    public void runOpMode() throws InterruptedException {
        AutonomousHardwareMap();
        waitForStart();
        AutonomousButtonPress("blue");
    }



}
