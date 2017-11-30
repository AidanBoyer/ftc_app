package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by agb on 1/13/2017.
 */

// @Autonomous(name = "Norm start, both balls, cap, both beacons")

public class AutoDoubleCapBoth extends LinearOpMode
{
    TouchSensor MRtouchSensor;
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;
    MasqAdafruitIMU imu;
    OpticalDistanceSensor ODS;
    UltrasonicSensor ultrasonicRangeSensor;

    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;
    byte[] colorSensorCache;

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

    public double angleDifference(double angleStarting, double angleTarget)
    {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(angleTarget-angleStarting)),Math.cos(Math.toRadians(angleTarget-angleStarting))));
    }

    public void turnUsingGyro(int turn)
    {
        double roughTolerance = 30;
        double fineTolerance = 0.3;

        double roughSpeed = 0.25;
        double fineSpeed = 0.10;
        double[] angles = imu.getAngularOrientation();

        double originalAngle = -angles[0];

        double targetAngle = originalAngle + turn;
        targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
    }

    public void turnToAngle(double targetAngle)
    {
        double roughTolerance = 30;
        double fineTolerance = 0.3;

        double roughSpeed = 0.25;
        double fineSpeed = 0.10;
        double[] angles = imu.getAngularOrientation();

        //double originalAngle = -angles[0];

        //double targetAngle = originalAngle + turn;
        //targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
    }

    public void fireLauncher()
    {
        int ticksPerLauncherRev = (int)(1478.4 * 1.4);
        LauncherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LauncherMotor.setTargetPosition(LauncherMotor.getCurrentPosition() + ticksPerLauncherRev);
        LauncherMotor.setPower(1);
        while(opModeIsActive() && LauncherMotor.isBusy()) {}
        LauncherMotor.setPower(0);
        LauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runDistanceWEncoders(double feet, double runningPower)
    {
        double inchesToTravel = feet * 12;
        double wheelDiameter = 3.75;
        double ticksPerRevolution = 1478.4;
        int ticksToTravel = (int)(inchesToTravel * ticksPerRevolution / (wheelDiameter * Math.PI));

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFrontDrive.setTargetPosition(RightFrontDrive.getCurrentPosition() + ticksToTravel);
        RightRearDrive.setTargetPosition(RightRearDrive.getCurrentPosition() + ticksToTravel);
        LeftFrontDrive.setTargetPosition(LeftFrontDrive.getCurrentPosition() + ticksToTravel);
        LeftRearDrive.setTargetPosition(LeftRearDrive.getCurrentPosition() + ticksToTravel);

        driveMotors(runningPower, runningPower, runningPower, runningPower);

        while(opModeIsActive() && RightRearDrive.isBusy() && LeftRearDrive.isBusy()) {}
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        boolean redAlliance = true;

        MRtouchSensor = hardwareMap.touchSensor.get("MR touch sensor");
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

        imu = new MasqAdafruitIMU("IMU", hardwareMap);
        ODS = hardwareMap.opticalDistanceSensor.get("Core ODS");
        ultrasonicRangeSensor = hardwareMap.ultrasonicSensor.get("ultrasonic range");

        double inchesBeforePressingBeacon = 5.5;
        int greenColorSensor = 6;
        double inchesBackingAfterFirstBeacon = 2.0;
        double ODSlineThreshold = 0.3;
        double firstInchesFromWall = 14;
        double secondInchesFromWall = 17;

        if(MRtouchSensor.isPressed())
        {
            redAlliance = false;
        }

        telemetry.addData("Red alliance: ", redAlliance);
        telemetry.update();

        waitForStart();

        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fireLauncher();
        sleep(200);
        runDistanceWEncoders(-1.0, -0.25);
        sleep(200);
        GathererMotor.setPower(1);
        sleep(1000);
        GathererMotor.setPower(0);
        sleep(200);
        GathererMotor.setPower(1);
        sleep(1000);
        GathererMotor.setPower(0);
        sleep(200);
        driveMotors(0.25, 0.25, 0.25, 0.25);
        sleep(1000);
        driveMotors(0, 0, 0, 0);
        sleep(200);
        fireLauncher();
        runDistanceWEncoders(-3.5, -0.5);
        sleep(500);
        runDistanceWEncoders(0.5, 0.25);
        sleep(200);
        // At this point, the leading edge of the robot should be 1 foot back from the center vortex base.

        if(redAlliance)
        {
            turnToAngle(290);
            sleep(100);
            runDistanceWEncoders(-2.0, -0.5);
            sleep(200);
            turnToAngle(0);
            sleep(100);
            runDistanceWEncoders(-2.0, -0.5);
            sleep(200);
            turnToAngle(270);
            sleep(100);
            while(ultrasonicRangeSensor.getUltrasonicLevel() > (firstInchesFromWall * 2.54) &&
                    ultrasonicRangeSensor.getUltrasonicLevel() != 0){
                driveMotors(-0.5, -0.5, -0.5, -0.5);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(0);
            sleep(100);
            while (ODS.getRawLightDetected() < 0.3) {
                driveMotors(-0.75, -0.75, -0.75, -0.75);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(270);
            sleep(100);
            runDistanceWEncoders(-((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)), -0.5*(Math.abs(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))/(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))));
            pushButton();
            sleep(5000);
            colorSensorCache = colorSensorReader.read(0x04, 1);
            if(colorSensorCache[0] < greenColorSensor && colorSensorCache[0] != 0)
            {
                pushButton();
            }

            while(ultrasonicRangeSensor.getUltrasonicLevel() < (secondInchesFromWall * 2.54) &&
                    ultrasonicRangeSensor.getUltrasonicLevel() != 255){
                driveMotors(0.5, 0.5, 0.5, 0.5);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(180);
            sleep(100);
            while (ODS.getRawLightDetected() < 0.3) {
                driveMotors(-0.75, -0.75, -0.75, -0.75);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(270);
            sleep(100);
            runDistanceWEncoders(-((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)), -0.5*(Math.abs(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))/(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))));
            pushButton();
            sleep(5000);
            colorSensorCache = colorSensorReader.read(0x04, 1);
            if(colorSensorCache[0] < greenColorSensor && colorSensorCache[0] != 0)
            {
                pushButton();
            }
        }

        else // if we're on the blue alliance
        {
            turnToAngle(70);
            sleep(100);
            runDistanceWEncoders(-2.0, -0.5);
            sleep(200);
            turnToAngle(0);
            sleep(100);
            runDistanceWEncoders(-2.0, -0.5);
            sleep(200);
            turnToAngle(90);
            sleep(100);
            while(ultrasonicRangeSensor.getUltrasonicLevel() > (firstInchesFromWall * 2.54) &&
                    ultrasonicRangeSensor.getUltrasonicLevel() != 0){
                driveMotors(-0.5, -0.5, -0.5, -0.5);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(0);
            sleep(100);
            while (ODS.getRawLightDetected() < 0.3) {
                driveMotors(-0.75, -0.75, -0.75, -0.75);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(90);
            sleep(100);
            runDistanceWEncoders(-((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)), -0.5*(Math.abs(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))/(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))));
            pushButton();
            sleep(5000);
            colorSensorCache = colorSensorReader.read(0x04, 1);
            if(colorSensorCache[0] > greenColorSensor && colorSensorCache[0] != 0)
            {
                pushButton();
            }

            while(ultrasonicRangeSensor.getUltrasonicLevel() < (secondInchesFromWall * 2.54) &&
                    ultrasonicRangeSensor.getUltrasonicLevel() != 255){
                driveMotors(0.5, 0.5, 0.5, 0.5);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(180);
            sleep(100);
            while (ODS.getRawLightDetected() < 0.3) {
                driveMotors(-0.75, -0.75, -0.75, -0.75);
            }
            shutOffMotors();
            sleep(200);
            turnToAngle(90);
            sleep(100);
            runDistanceWEncoders(-((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)), -0.5*(Math.abs(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))/(((ultrasonicRangeSensor.getUltrasonicLevel() * 2.54) - (inchesBeforePressingBeacon / 12)))));
            pushButton();
            sleep(5000);
            colorSensorCache = colorSensorReader.read(0x04, 1);
            if(colorSensorCache[0] > greenColorSensor && colorSensorCache[0] != 0)
            {
                pushButton();
            }
        }
    }
}
