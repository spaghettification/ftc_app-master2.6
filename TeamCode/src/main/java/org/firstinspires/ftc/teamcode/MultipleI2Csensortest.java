package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Trevor on 12/22/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Color sensor Test", group = "Velocity Vortex Autonomous")
public class MultipleI2Csensortest extends LinearHardwareMap {

    ColorSensor C1;
    ColorSensor C2;

    @Override
    public void runOpMode() throws InterruptedException {
        C1 = hardwareMap.colorSensor.get("bcs");
        C1.setI2cAddress(I2cAddr.create7bit(0x1e));
        C1.enableLed(false);
        C2 = hardwareMap.colorSensor.get("bcs2");
        C2.setI2cAddress(I2cAddr.create7bit(0x26));
        C2.enableLed(true);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("1, blue", C1.blue());
            telemetry.addData("1, red", C1.red());
            telemetry.addData("1, green", C1.green());
            telemetry.addData("2, blue", C2.blue());
            telemetry.addData("2, red", C2.red());
            telemetry.addData("2, green", C2.green());
            telemetry.update();
        }

    }
}
