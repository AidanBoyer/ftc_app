package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by agb on 1/14/2017.
 */

@Autonomous(name = "Fire two balls")
@Disabled

public class fireTwoBalls extends LinearOpMode {
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor GathererMotor;
    DcMotor LauncherMotor;

    public void fireLauncher()
    {
        int ticksPerLauncherRev = (int)(1478.4 * 1.4);
        LauncherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LauncherMotor.setTargetPosition(LauncherMotor.getCurrentPosition() + ticksPerLauncherRev);
        LauncherMotor.setPower(1);
        while(opModeIsActive() && LauncherMotor.isBusy()) {}
        LauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

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

        while(opModeIsActive() && RightRearDrive.isBusy() && LeftRearDrive.isBusy() && RightFrontDrive.isBusy() && LeftRearDrive.isBusy()){}
        shutOffMotors();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        waitForStart();
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
    }
}
