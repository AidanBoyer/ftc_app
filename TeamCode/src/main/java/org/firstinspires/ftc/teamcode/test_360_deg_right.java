package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by agb on 1/1/2018.
 */

@Autonomous(name = "test 360 degrees positive")

public class test_360_deg_right extends LinearOpMode
{
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;

    MasqAdafruitIMU imu;

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



    @Override
    public void runOpMode() throws InterruptedException {

        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");

        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new MasqAdafruitIMU("IMU", hardwareMap);

        waitForStart();

        turnUsingGyro(180);
    }

}
