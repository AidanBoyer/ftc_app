package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by agb on 1/12/2017.
 */

@Autonomous(name = "Push beacon: start slightly >1ft away")

public class pressBeacon extends LinearOpMode {

    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;

    byte[] colorSensorCache;

    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;

    public void shutOffMotors()
    {
        driveMotors(0, 0, 0, 0);
    }

    public void setDriveRunMode(DcMotor.RunMode runMode)
    {
        RightFrontDrive.setMode(runMode);
        RightRearDrive.setMode(runMode);
        LeftFrontDrive.setMode(runMode);
        LeftRearDrive.setMode(runMode);
    }

    public void driveMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        RightFrontDrive.setPower(rightFront);
        RightRearDrive.setPower(rightRear);
        LeftFrontDrive.setPower(leftFront);
        LeftRearDrive.setPower(leftRear);
    }

    public void pushButton()
    {
        driveMotors(-0.5, -0.5, -0.5, -0.5);

        sleep(500);

        shutOffMotors();

        sleep(200);

        driveMotors(0.2, 0.2, 0.2, 0.2);

        sleep(200);

        shutOffMotors();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        GathererMotor = hardwareMap.dcMotor.get("gatherer motor");
        LauncherMotor = hardwareMap.dcMotor.get("launcher motor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GathererMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwareMap.i2cDevice.get("MR color sensor");
        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        colorSensorReader.engage();
        colorSensorReader.write8(3, 1);

        double feetToTravel = -1.0; // ENTER FEET TO TRAVEL HERE
        double runningPower = -0.5;

        double inchesToTravel = feetToTravel * 12;
        double wheelDiameter = 3.75;
        double ticksPerRevolution = 1478.4;
        int ticksToTravel = (int)(inchesToTravel * ticksPerRevolution / (wheelDiameter * 3.14159));

        waitForStart();

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFrontDrive.setTargetPosition(RightFrontDrive.getCurrentPosition() + ticksToTravel);
        RightRearDrive.setTargetPosition(RightRearDrive.getCurrentPosition() + ticksToTravel);
        LeftFrontDrive.setTargetPosition(LeftFrontDrive.getCurrentPosition() + ticksToTravel);
        LeftRearDrive.setTargetPosition(LeftRearDrive.getCurrentPosition() + ticksToTravel);

        driveMotors(runningPower, runningPower, runningPower, runningPower);

        while(opModeIsActive() && RightFrontDrive.isBusy()) {}

        shutOffMotors();

        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(500);

        pushButton();

        sleep(5000);

        colorSensorCache = colorSensorReader.read(0x04, 1);

        // FOR RED

        if(colorSensorCache[0] < 5 && colorSensorCache[0] != 0)
        {
            pushButton();
        }
    }
}
