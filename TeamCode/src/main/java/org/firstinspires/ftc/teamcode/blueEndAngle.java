package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by agb on 1/18/2018.
 */

@Autonomous(name = "End, angle", group = "Autonomous")

public class blueEndAngle extends LinearOpMode
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
    //ModernRoboticsI2cCompassSensor compassSensor;

    MasqAdafruitIMU imu;
    VuforiaLocalizer vuforia;

    public void driveMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        RightFrontDrive.setPower(rightFront);
        RightRearDrive.setPower(rightRear);
        LeftFrontDrive.setPower(leftFront);
        LeftRearDrive.setPower(leftRear);
    }

    public void moveAllMotors(double power)
    {
        RightFrontDrive.setPower(power);
        RightRearDrive.setPower(power);
        LeftFrontDrive.setPower(power);
        LeftRearDrive.setPower(power);
    }

    public void accelerateGradually(int increments, int targetPower)
    {
        int multiplier = targetPower / increments;
        for(int i = 1; i <= increments; i++)
        {
            moveAllMotors((i * multiplier) / 100);
            sleep(20);
        }
    }

    public void driveRightMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        RightFrontDrive.setPower(rightFront);
        RightRearDrive.setPower(rightRear);
        //LeftFrontDrive.setPower(leftFront);
        //LeftRearDrive.setPower(leftRear);
    }

    public void driveLeftMotors(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        //RightFrontDrive.setPower(rightFront);
        //RightRearDrive.setPower(rightRear);
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

    public void turnUsingGyroLeftOnly(int turn, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

        double[] angles = imu.getAngularOrientation();

        double originalAngle = -angles[0];

        double targetAngle = originalAngle + turn;
        targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveLeftMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveLeftMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
    }

    public void turnToAngle(double targetAngle, double roughSpeed, double roughTolerance, double fineSpeed, double fineTolerance)
    {

        double[] angles = imu.getAngularOrientation();

        //double originalAngle = -angles[0];

        //double targetAngle = originalAngle + turn;
        //targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()) {
            int sign = (int) (angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        double angle0 = 0;
        for (int ii = 0; ii < 10; ii++)
        {
            angles = imu.getAngularOrientation();
            angle0 = angle0 + angles[0];
        }
        angle0 = angle0 / 10;


        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            for (int ii = 0; ii < 10; ii++)
            {
                angles = imu.getAngularOrientation();
                angle0 = angle0 + angles[0];
            }
            angle0 = angle0 / 10;
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

    public void turnToAngleLeftOnly(double targetAngle, double roughSpeed, double fineSpeed, double fineTolerance)
    {
        double roughTolerance = 30;

        double[] angles = imu.getAngularOrientation();

        //double originalAngle = -angles[0];

        //double targetAngle = originalAngle + turn;
        //targetAngle = targetAngle % 360;
        //if (turn<0) {targetAngle = -1*targetAngle;}

        while((Math.abs(angleDifference(-angles[0], targetAngle)) > roughTolerance) && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveLeftMotors(sign * -roughSpeed, sign * -roughSpeed, sign * roughSpeed, sign * roughSpeed);
            telemetry.addData("Coarse", imu.telemetrize());
            telemetry.addData("Angle difference", angleDifference(-angles[0], targetAngle));
            telemetry.update();

            angles = imu.getAngularOrientation();
        }

        while(Math.abs(angleDifference(-angles[0], targetAngle)) > fineTolerance && opModeIsActive()){
            int sign = (int)(angleDifference(-angles[0], targetAngle) / Math.abs(angleDifference(-angles[0], targetAngle)));
            driveLeftMotors(sign * -fineSpeed, sign * -fineSpeed, sign * fineSpeed, sign * fineSpeed);
            telemetry.addData("Fine", imu.telemetrize());
            telemetry.update();

            angles = imu.getAngularOrientation();
        }
        shutOffMotors();
    }

    public void runDistanceWEncoders(double feet, double runningPower, boolean shutoffWhenDone)
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
        if(shutoffWhenDone){shutOffMotors();}
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean redAlliance = true;
        teamSelector = hardwareMap.touchSensor.get("team selector button");
        if(teamSelector.isPressed())
        {
            redAlliance = false;
            telemetry.addLine("Blue alliance");
        }
        else{telemetry.addLine("Red alliance");}

        double leftGripperHoldingOffsetFromHalf = 0.030; // smaller number is closed tighter
        double rightGripperHoldingOffsetFromHalf = 0.001; // larger number is closed tighter
        //double jewelArmServoStartingPosition = 0.07;
        double jewelArmServoRestPosition = 0.19;
        double jewelArmServoDownPosition = 0.865;
        double gripperHoldingSqueeze = 0.035;

        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        LiftMotor = hardwareMap.dcMotor.get("lift motor");

        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        JewelArmServo = hardwareMap.servo.get("jewel arm");

        ServoController1 = hardwareMap.servoController.get("servo controller 1");
        ServoController1.pwmDisable();

        armSafetyStop = hardwareMap.touchSensor.get("arm safety stop");

        JewelColorSensor = hardwareMap.colorSensor.get("jewel color sensor");
        imu = new MasqAdafruitIMU("IMU", hardwareMap);
        //compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass sensor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        JewelColorSensor.enableLed(true);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AQvbmU7/////AAAAGb+OYMKGPUZOr+k1NC6Kbih5aUKWLiVHtSwp8Gcl1zv/u0DD5aiq75Z2T/KzneDb+SkazMqa5EohmpUcSzjRKUIDnftPjJpaU/6uxtr2g/Tr/WYSi+x8sr4odiVJMmlYlPlpTHSL1SPs1s0+s2DfAEP4J9Qdzix4y6+CLDnNR5yKN2j+7YfzRpJNRBfItD2GhU6G6tq3Dkv7xZcOwlyqlxqAOZ3+MU4fB1Hgit/ivliXdnm1nhWo38y8orWR2vzRDWdfIlu9ERn9zy0UnVmQJbPKs4euZU3zanE9UnxREnqFy3rpuV5xxhgj1LgDzfF9C+YwWjXYB6rXtADfdg76hZnKC1FUX3P8oWxrQRFqtojK";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.update();

        waitForStart(); // _________________________________________________________________________

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        while(vuMark == RelicRecoveryVuMark.UNKNOWN)
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        JewelArmServo.setPosition(jewelArmServoRestPosition);

        LeftGripperServo.setPosition(0.5 + leftGripperHoldingOffsetFromHalf);
        RightGripperServo.setPosition(0.5 + rightGripperHoldingOffsetFromHalf);

        LeftGripperServo.setPosition(0.5 + leftGripperHoldingOffsetFromHalf - gripperHoldingSqueeze);
        RightGripperServo.setPosition(0.5 + rightGripperHoldingOffsetFromHalf + gripperHoldingSqueeze);

        sleep(300);

        LiftMotor.setTargetPosition(10000);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(1);

        sleep(50);

        runDistanceWEncoders(0.15, 0.15, true);

        sleep(50);

        JewelArmServo.setPosition(jewelArmServoDownPosition);
        sleep(500);

        if(redAlliance)
        {
            if(JewelColorSensor.red() < JewelColorSensor.blue())
            {
                //turnUsingGyroRightOnly(12, 0.25, 0.09, 1); // turn
                runDistanceWEncoders(0.25,0.2,true);
                runDistanceWEncoders(-0.25, -0.2, true);
            }
            else
            {
                //turnUsingGyroRightOnly(-12, 0.25, 0.09, 1); // turn left
                runDistanceWEncoders(-0.25, -0.2, true);
                runDistanceWEncoders(0.25, 0.2, true);
            }

            //sleep(500);
            JewelArmServo.setPosition(jewelArmServoRestPosition);
            //turnToAngleRightOnly(0, 0.25, 0.08, 0.2);
            sleep(50);

            runDistanceWEncoders(-1.85, -0.30, true);
            sleep(50);
            driveMotors(0,0.25,0,0.25);
            sleep(1000);
            shutOffMotors();

            runDistanceWEncoders(-1.2, -0.35, true); // this block reverses the robot so it's in the same position as when it goes off facing forward
            sleep(50);
            turnToAngle(180, 0.35,18,0.06,0.2);
            sleep(50);
            driveMotors(-0.15, -0.15, -0.15, -0.15);
            sleep(1000);
            shutOffMotors();

            switch(vuMark)
            {
                case LEFT:
                    runDistanceWEncoders(0.25, 0.20, true);
                    sleep(50);
                    turnToAngle(-270, 0.30, 18, 0.06, 0.2);
                    sleep(50);
                    runDistanceWEncoders(1.15, 0.35, true);
                    sleep(50);
                    turnToAngle(-210,0.30,18,0.06,0.2);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    runDistanceWEncoders(-0.15, -0.2, true);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(200);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    moveAllMotors(0.25);
                    sleep(1500);
                    break;
                case CENTER:
                    turnToAngleRightOnly(-226,0.30,0.08,0.2);

                    sleep(50);

                    runDistanceWEncoders(0.2, 0.20, true);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    runDistanceWEncoders(-0.1, -0.20, true);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(300);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    accelerateGradually(25, 30);
                    moveAllMotors(0.3);
                    sleep(1000);
                    shutOffMotors();
                    break;
                case RIGHT:

                    turnToAngleRightOnly(-202,0.30,0.08,0.2);
                    sleep(50);

                    runDistanceWEncoders(0.2, 0.20, true);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    runDistanceWEncoders(-0.1, -0.20, true);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(300);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    accelerateGradually(25, 30);
                    moveAllMotors(0.3);
                    sleep(1000);
                    shutOffMotors();
                    break;
            }
        }
        else // if blue alliance
        {
            if(JewelColorSensor.red() < JewelColorSensor.blue())
            {
                //turnUsingGyroRightOnly(-12, 0.25, 0.09, 0.3); // turn left
                runDistanceWEncoders(-0.25, -0.2, true);
                runDistanceWEncoders(0.25, 0.2, true);
            }
            else
            {
                //turnUsingGyroRightOnly(12, 0.25, 0.09, 0.3); // turn right
                runDistanceWEncoders(0.25, 0.2, true);
                runDistanceWEncoders(-0.25, -0.2, true);
            }

            //sleep(500);
            JewelArmServo.setPosition(jewelArmServoRestPosition);
            //turnToAngleRightOnly(0, 0.25, 0.08, 0.2);
            sleep(200);

            runDistanceWEncoders(1.78, 0.20, true);
            sleep(200);
            driveMotors(-0.15, -0.15, -0.15, -0.15);
            sleep(1000);
            shutOffMotors();

            sleep(300);

            switch(vuMark)
            {
                case LEFT:
                    turnToAngleLeftOnly(24,0.30,0.08,0.2);

                    sleep(200);

                    runDistanceWEncoders(0.2, 0.15, true);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    runDistanceWEncoders(-0.1, -0.15, true);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(300);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    sleep(200);
                    accelerateGradually(25, 30);
                    moveAllMotors(0.3);
                    sleep(1000);
                    shutOffMotors();
                    break;

                case CENTER:
                    turnToAngleLeftOnly(46,0.30,0.08,0.2);

                    sleep(200);

                    runDistanceWEncoders(0.2, 0.15, true);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    runDistanceWEncoders(-0.1, -0.15, true);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(300);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    sleep(200);
                    accelerateGradually(25, 30);
                    moveAllMotors(0.3);
                    sleep(1000);
                    shutOffMotors();
                    break;

                case RIGHT:
                    runDistanceWEncoders(0.25, 0.15, true);
                    sleep(300);
                    turnToAngle(90, 0.30, 18, 0.06, 0.2);
                    sleep(200);
                    runDistanceWEncoders(1.25, 0.30, true);
                    sleep(300);
                    turnToAngle(30,0.30,18,0.06,0.2);
                    sleep(200);

                    LiftMotor.setTargetPosition(0);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(-1);

                    while(opModeIsActive() && LiftMotor.isBusy()) {} // don't continue until the left comes down

                    LeftGripperServo.setPosition(0.75);
                    RightGripperServo.setPosition(0.25);

                    moveAllMotors(0.25);

                    LiftMotor.setTargetPosition(800);
                    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LiftMotor.setPower(1);

                    sleep(200);

                    LeftGripperServo.setPosition(1);
                    RightGripperServo.setPosition(0);

                    sleep(500);
                    shutOffMotors();
                    runDistanceWEncoders(-0.15, -0.2, true);
                    moveAllMotors(0.25);
                    sleep(700);
            }
        }
        runDistanceWEncoders(-0.15, -0.2, true);
    }
}
