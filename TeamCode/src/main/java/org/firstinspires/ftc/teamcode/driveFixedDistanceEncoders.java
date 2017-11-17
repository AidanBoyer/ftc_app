package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 1/10/2017.
 */

@Autonomous(name = "Drive set distance using encoders")

public class driveFixedDistanceEncoders extends LinearOpMode
{
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;

    public void moveAllMotors(double power)
    {
        RightFrontDrive.setPower(power);
        RightRearDrive.setPower(power);
        LeftFrontDrive.setPower(power);
        LeftRearDrive.setPower(power);
    }

    public void accelerateGradually()
    {
        for(int i = 1; i <= 25; i = i + 1)
        {
            moveAllMotors((i * 4) / 100);
            sleep(20);
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        double feetToTravel = 3.0; // ENTER FEET TO TRAVEL HERE
        double runningPower = 0.5;

        double inchesToTravel = feetToTravel * 12;
        double wheelDiameter = 3.75;
        double ticksPerRevolution = 1478.4;
        int ticksToTravel = (int)(inchesToTravel * ticksPerRevolution / (wheelDiameter * 3.14159));

        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        GathererMotor = hardwareMap.dcMotor.get("gatherer motor");
        LauncherMotor = hardwareMap.dcMotor.get("launcher motor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GathererMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        RightFrontDrive.setTargetPosition(RightFrontDrive.getCurrentPosition() + ticksToTravel);
        RightRearDrive.setTargetPosition(RightRearDrive.getCurrentPosition() + ticksToTravel);
        LeftFrontDrive.setTargetPosition(LeftFrontDrive.getCurrentPosition() + ticksToTravel);
        LeftRearDrive.setTargetPosition(LeftRearDrive.getCurrentPosition() + ticksToTravel);

        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFrontDrive.setPower(runningPower);
        RightRearDrive.setPower(runningPower);
        LeftFrontDrive.setPower(runningPower);
        LeftRearDrive.setPower(runningPower);

        while(opModeIsActive() && RightFrontDrive.isBusy() && RightRearDrive.isBusy() && LeftFrontDrive.isBusy() && LeftRearDrive.isBusy()) {}
    }
}
