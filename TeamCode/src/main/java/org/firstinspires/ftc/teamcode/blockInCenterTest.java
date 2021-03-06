package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by agb on 1/9/2018.
 */

@Autonomous(name = "block, jewel, center")
@Disabled

public class blockInCenterTest extends LinearOpMode
{
    TouchSensor teamSelector;
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor LiftMotor;

    Servo LeftGripperServo;
    Servo RightGripperServo;
    Servo JewelArmServo;

    ServoController ServoController1;

    TouchSensor armSafetyStop;
    ColorSensor JewelColorSensor;
    ModernRoboticsI2cCompassSensor compassSensor;

    MasqAdafruitIMU imu;

    public void driveMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        RightFrontDrive.setPower(rightFront);
        RightRearDrive.setPower(rightRear);
        LeftFrontDrive.setPower(leftFront);
        LeftRearDrive.setPower(leftRear);
    }

    public void driveRightMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        RightFrontDrive.setPower(rightFront);
        RightRearDrive.setPower(rightRear);
        LeftFrontDrive.setPower(leftFront);
        LeftRearDrive.setPower(leftRear);
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

    public double angleDifference(double angleStarting, double angleTarget)
    {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(angleTarget-angleStarting)),Math.cos(Math.toRadians(angleTarget-angleStarting))));
    }

    public void turnUsingGyro(int turn, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

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

    public void turnUsingGyroRightOnly(int turn, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

        double[] angles = imu.getAngularOrientation();

        double originalAngle = -angles[0];

        double targetAngle = originalAngle + turn;
        targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveRightMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveRightMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
    }

    public void turnToAngle(double targetAngle, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

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

    public void turnToAngleRightOnly(double targetAngle, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

        double[] angles = imu.getAngularOrientation();

        //double originalAngle = -angles[0];

        //double targetAngle = originalAngle + turn;
        //targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveRightMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveRightMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
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

        double leftGripperHoldingOffsetFromHalf = 0.030; // smaller number is closed tighter
        double rightGripperHoldingOffsetFromHalf = 0.001; // larger number is closed tighter
        double jewelArmServoStartingPosition = 0.07;
        double jewelArmServoRestPosition = 0.19;
        double jewelArmServoDownPosition = 0.865;
        double gripperHoldingSqueeze = 0.035;

        boolean redAlliance = true;
        teamSelector = hardwareMap.touchSensor.get("team selector button");
        if(teamSelector.isPressed())
        {
            redAlliance = false;
            telemetry.addLine("Blue alliance");
        }
        else{telemetry.addLine("Red alliance");}

        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        LiftMotor = hardwareMap.dcMotor.get("lift motor");


        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        JewelArmServo = hardwareMap.servo.get("jewel arm");

        ServoController1 = hardwareMap.servoController.get("servo controller 1");

        armSafetyStop = hardwareMap.touchSensor.get("arm safety stop");

        JewelColorSensor = hardwareMap.colorSensor.get("jewel color sensor");
        imu = new MasqAdafruitIMU("IMU", hardwareMap);
        compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass sensor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoController1.pwmDisable();

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        JewelColorSensor.enableLed(true);

        telemetry.update();

        waitForStart();

        JewelArmServo.setPosition(jewelArmServoRestPosition);

        LeftGripperServo.setPosition(0.5 + leftGripperHoldingOffsetFromHalf);
        RightGripperServo.setPosition(0.5 + rightGripperHoldingOffsetFromHalf);

        sleep(100);

        LeftGripperServo.setPosition(0.5 + leftGripperHoldingOffsetFromHalf - gripperHoldingSqueeze);
        RightGripperServo.setPosition(0.5 + rightGripperHoldingOffsetFromHalf + gripperHoldingSqueeze);

        sleep(100);

        LiftMotor.setTargetPosition(8000);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(1);

        //while(opModeIsActive() && LiftMotor.isBusy()) {}

        JewelArmServo.setPosition(jewelArmServoDownPosition);
        sleep(500);
        if(redAlliance)
        {
            if(JewelColorSensor.red() > JewelColorSensor.blue())
            {
                turnUsingGyroRightOnly(10, 0.25, 0.09, 0.3); // turn right
            }
            else
            {
                turnUsingGyroRightOnly(-10, 0.25, 0.09, 0.3); // turn left
            }
        }
        else // if blue alliance
        {
            if(JewelColorSensor.red() > JewelColorSensor.blue())
            {
                turnUsingGyroRightOnly(-10, 0.25, 0.09, 0.3); // turn left
            }
            else
            {
                turnUsingGyroRightOnly(10, 0.25, 0.09, 0.3); // turn right
            }
        }
        sleep(500);
        JewelArmServo.setPosition(jewelArmServoRestPosition);
        turnToAngleRightOnly(0, 0.25, 0.08, 0.2);
        sleep(500);
        runDistanceWEncoders(2.5, 0.30);
        sleep(500);
        turnToAngle(-90,0.25, 0.09, 0.2);

        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(-1);

        while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

        LeftGripperServo.setPosition(0.75);
        RightGripperServo.setPosition(0.25);

        sleep(200);

        LiftMotor.setTargetPosition(800);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(1);

        LeftGripperServo.setPosition(1);
        RightGripperServo.setPosition(0);

        runDistanceWEncoders(-0.1, -0.15);
        runDistanceWEncoders(0.1, 0.15);

        driveMotors(0.3, 0.3, 0.3, 0.3);
        sleep(1500);
        shutOffMotors();
    }
}
