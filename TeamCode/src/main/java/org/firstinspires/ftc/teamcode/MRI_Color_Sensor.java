package org.firstinspires.ftc.teamcode;

/*
Modern Robotics Color Sensor Example with color number
Created 9/29/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "cc" (MRI Color Sensor with default I2C address 0x3c (0x1e 7-bit)

ModernRoboticsI2cColorSensor class is not being used because it can not access color number.
ColorSensor class is not being used because it can not access color number.

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Color Sensor", group = "MRI")

public class MRI_Color_Sensor extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorSensorCache;

    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorSensor = hardwareMap.i2cDevice.get("MR color sensor");
        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        colorSensorReader.engage();
    }

    @Override
    public void start()
    {
        runtime.reset();

        colorSensorReader.write8(3, 1);
    }

    @Override
    public void loop()
    {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorSensorCache = colorSensorReader.read(0x04, 1);

        //display values
        telemetry.addData("2 #C", colorSensorCache[0] & 0xFF);
    }
}