package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by agb on 12/7/2017.
 */

@TeleOp(name = "1/18/18", group = "TeleOp")

public class teleOpWithLiftDestinations extends OpMode
{

    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightRearDrive;
    DcMotor LeftRearDrive;
    DcMotor LiftMotor;

    Servo LeftGripperServo;
    Servo RightGripperServo;
    Servo JewelArmServo;

    ServoController ServoController1;
    //DcMotorController RightDriveController;
    //DcMotorController LeftDriveController;

    TouchSensor armSafetyStop;
    //ModernRoboticsI2cCompassSensor compassSensor;
    ColorSensor JewelColorSensor;
    MasqAdafruitIMU imu;

    @Override
    public void init() {
        RightFrontDrive = hardwareMap.dcMotor.get("right front drive");
        LeftFrontDrive = hardwareMap.dcMotor.get("left front drive");
        RightRearDrive = hardwareMap.dcMotor.get("right rear drive");
        LeftRearDrive = hardwareMap.dcMotor.get("left rear drive");
        LiftMotor = hardwareMap.dcMotor.get("lift motor");

        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        JewelArmServo = hardwareMap.servo.get("jewel arm");

        ServoController1 = hardwareMap.servoController.get("servo controller 1");

        JewelArmServo.setPosition(jewelArmServoRestPosition);
        LeftGripperServo.setPosition(1);
        RightGripperServo.setPosition(0);

        ServoController1.pwmDisable();

        //RightDriveController = hardwareMap.dcMotorController.get("right drive motor controller");
        //LeftDriveController = hardwareMap.dcMotorController.get("left drive motor controller");

        armSafetyStop = hardwareMap.touchSensor.get("arm safety stop");

        //compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass sensor");
        JewelColorSensor = hardwareMap.colorSensor.get("jewel color sensor");

        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        JewelColorSensor.enableLed(false);

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = new MasqAdafruitIMU("IMU", hardwareMap);
    }

    double leftGripperHoldingOffsetFromHalf = 0.030; // smaller number is closed tighter
    double rightGripperHoldingOffsetFromHalf = 0.001; // larger number is closed tighter
    double squeezingDownsizeCoefficient = 200;
    double gripperHoldingSqueeze = 0.035;
    double dPadTurnSpeed = 0.60;

    int liftTicksPickup = 0;
    int liftTicksSecond = 3000;
    int liftTicksThird = 5700;
    int liftTicksFourth = 9100;

    //double jewelArmServoStartingPosition = 0.07;
    double jewelArmServoRestPosition = 0.19;
    //double jewelArmServoDownPosition = 0.865;

    boolean arcadeDriveStyle = false;
    boolean driveStyleWasPressed = false;
    boolean driveReversed = false;
    boolean wasReverseButton = false;
    boolean absoluteServoControl = false;
    boolean guideWasPressed = false;

    //double leftGripperStartingPosition;
    //double rightGripperStartingPosition;
    //double leftGripperOpenedPosition;
    //double rightGripperOpenedPosition;
    double rightFrontDriveValue = 0;
    double rightRearDriveValue = 0;
    double leftFrontDriveValue = 0;
    double leftRearDriveValue = 0;
    double liftMotorValue = 0;
    double rightGripperPosition = 0.5 + rightGripperHoldingOffsetFromHalf;
    double leftGripperPosition = 0.5 + leftGripperHoldingOffsetFromHalf;

    @Override
    public void loop()
    {
        // GAMEPAD ONE __________________________________________________________________________________________

        rightFrontDriveValue = 0;
        rightRearDriveValue = 0;
        leftFrontDriveValue = 0;
        leftRearDriveValue = 0;
        liftMotorValue = 0;

        if(gamepad1.guide)
        {
            if(!driveStyleWasPressed) // if the button wasn't pressed last time
            {
                if (arcadeDriveStyle) {arcadeDriveStyle = false;} // if arcadeDriveStyle is true, make it false
                else if (!arcadeDriveStyle) {arcadeDriveStyle = true;} // if arcadeDriveStyle is false, make it true
                driveStyleWasPressed = true;
            }
        }
        else // if the button isn't pressed
        {
            driveStyleWasPressed = false;
        }

        if(arcadeDriveStyle)
        {
            float xValue;
            float yValue;

            if (gamepad1.left_stick_x > 0) {
                xValue = gamepad1.left_stick_x * gamepad1.left_stick_x;
            } else {
                xValue = -gamepad1.left_stick_x * gamepad1.left_stick_x;
            }

            if (-gamepad1.left_stick_y > 0) {
                yValue = -gamepad1.left_stick_y * -gamepad1.left_stick_y;
            } else {
                yValue = gamepad1.left_stick_y * -gamepad1.left_stick_y;
            }

            float rightPower = yValue - xValue;
            float leftPower = yValue + xValue;

            rightPower = Range.clip(rightPower, -1, 1);
            leftPower = Range.clip(leftPower, -1, 1);

            rightFrontDriveValue = rightPower;
            rightRearDriveValue = rightPower;
            leftFrontDriveValue = leftPower;
            leftRearDriveValue = leftPower;
        }

        if(!arcadeDriveStyle)
        {
            double rightStickVal = -gamepad1.right_stick_y;
            double leftStickVal = -gamepad1.left_stick_y;
            double rightSquaredVal = rightStickVal * rightStickVal;
            double leftSquaredVal = leftStickVal * leftStickVal;

            if(rightStickVal < 0) {
                rightFrontDriveValue = -rightSquaredVal;
                rightRearDriveValue = -rightSquaredVal;
            } else {
                rightFrontDriveValue = rightSquaredVal;
                rightRearDriveValue = rightSquaredVal;
            }
            if(leftStickVal < 0) {
                leftFrontDriveValue = -leftSquaredVal;
                leftRearDriveValue = -leftSquaredVal;
            } else {
                leftFrontDriveValue = leftSquaredVal;
                leftRearDriveValue = leftSquaredVal;
            }
        }

        if (gamepad1.right_trigger != 0)
        {
            rightFrontDriveValue = gamepad1.right_trigger;
            rightRearDriveValue = gamepad1.right_trigger;
            leftFrontDriveValue = gamepad1.right_trigger;
            leftRearDriveValue = gamepad1.right_trigger;
        }
        else if (gamepad1.left_trigger != 0)
        {
            rightFrontDriveValue = -gamepad1.left_trigger;
            rightRearDriveValue = -gamepad1.left_trigger;
            leftFrontDriveValue = -gamepad1.left_trigger;
            leftRearDriveValue = -gamepad1.left_trigger;
        }

        if(gamepad1.dpad_left)
        {
            rightFrontDriveValue = dPadTurnSpeed;
            rightRearDriveValue = dPadTurnSpeed;
            leftRearDriveValue = -dPadTurnSpeed;
            leftFrontDriveValue = -dPadTurnSpeed;
        }
        if(gamepad1.dpad_right)
        {
            rightFrontDriveValue = -dPadTurnSpeed;
            rightRearDriveValue = -dPadTurnSpeed;
            leftRearDriveValue = dPadTurnSpeed;
            leftFrontDriveValue = dPadTurnSpeed;
        }

        if(!gamepad1.left_bumper)
        {
            rightFrontDriveValue = rightFrontDriveValue / 3;
            rightRearDriveValue = rightRearDriveValue / 3;
            leftFrontDriveValue = leftFrontDriveValue / 3;
            leftRearDriveValue = leftRearDriveValue / 3;
        }

        if(gamepad1.right_bumper)
        {
            if(!wasReverseButton)
            {
                if(!driveReversed) {driveReversed = true;}
                else {driveReversed = false;}
            }
            wasReverseButton = true;
        }
        else
        {
            wasReverseButton = false;
        }

        if(driveReversed)
        {
            double RR = rightRearDriveValue;
            double LR = leftRearDriveValue;
            double RF = rightFrontDriveValue;
            double LF = leftFrontDriveValue;

            leftRearDriveValue = RR * -1;
            rightRearDriveValue = LR * -1;
            leftFrontDriveValue = RF * -1;
            rightFrontDriveValue = LF * -1;
        }

        // GAMEPAD TWO _________________________________________________________________________________________

        if(gamepad2.guide)
        {
            if(!guideWasPressed) // if the button wasn't pressed last time
            {
                if (absoluteServoControl) {absoluteServoControl = false;} // if absoluteServoControl is true, make it false
                else if (!absoluteServoControl) {absoluteServoControl = true;} // if absoluteServoControl is false, make it true
                guideWasPressed = true;
            }
        }
        else {guideWasPressed = false;}

        if(absoluteServoControl)
        {
            rightGripperPosition = (((((gamepad2.right_stick_x * gamepad2.right_stick_x * Math.signum(gamepad2.right_stick_x)) * -1) + 1) / 2) + rightGripperHoldingOffsetFromHalf);
            leftGripperPosition = (((((gamepad2.left_stick_x * gamepad2.left_stick_x * Math.signum(gamepad2.left_stick_x)) * -1) + 1) / 2) + leftGripperHoldingOffsetFromHalf);

            if(LiftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && ((gamepad2.right_trigger != 0) || (gamepad2.left_trigger != 0)))
            {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            liftMotorValue = ((gamepad2.right_trigger * gamepad2.right_trigger) - (gamepad2.left_trigger * gamepad2.left_trigger));
        }
        else
        {
            if(LiftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && (gamepad2.right_stick_y != 0))
            {
                LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            liftMotorValue = gamepad2.right_stick_y * gamepad2.right_stick_y * -Math.signum(gamepad2.right_stick_y);

            leftGripperPosition = leftGripperPosition - (gamepad2.right_trigger / squeezingDownsizeCoefficient) + (gamepad2.left_trigger / squeezingDownsizeCoefficient);
            rightGripperPosition = rightGripperPosition + (gamepad2.right_trigger / squeezingDownsizeCoefficient) - (gamepad2.left_trigger / squeezingDownsizeCoefficient);
        }

        if(gamepad2.back)
        {
            rightGripperPosition = 0.5 + rightGripperHoldingOffsetFromHalf;
            leftGripperPosition = 0.5 + leftGripperHoldingOffsetFromHalf;
        }

        if(gamepad2.start)
        {
            leftGripperPosition = (0.5 + leftGripperHoldingOffsetFromHalf - gripperHoldingSqueeze);
            rightGripperPosition = (0.5 + rightGripperHoldingOffsetFromHalf + gripperHoldingSqueeze);
        }

        if(gamepad2.a)
        {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setTargetPosition(liftTicksPickup);
            if (LiftMotor.getCurrentPosition() > liftTicksPickup) {
                LiftMotor.setPower(-1);
            } else {
                LiftMotor.setPower(1);
            }
        }
        if(gamepad2.x)
        {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setTargetPosition(liftTicksSecond);
            if (LiftMotor.getCurrentPosition() > liftTicksSecond) {
                LiftMotor.setPower(-1);
            } else {
                LiftMotor.setPower(1);
            }
        }
        if(gamepad2.y)
        {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setTargetPosition(liftTicksThird);
            if (LiftMotor.getCurrentPosition() > liftTicksThird) {
                LiftMotor.setPower(-1);
            } else {
                LiftMotor.setPower(1);
            }
        }
        if(gamepad2.b)
        {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setTargetPosition(liftTicksFourth);
            if (LiftMotor.getCurrentPosition() > liftTicksFourth) {
                LiftMotor.setPower(-1);
            } else {
                LiftMotor.setPower(1);
            }
        }

        // _______________________________________________________________________________________

        if(armSafetyStop.isPressed() && (Math.signum(liftMotorValue) == 1))
        {
            liftMotorValue = 0;
        }

        RightFrontDrive.setPower(rightFrontDriveValue);
        RightRearDrive.setPower(rightRearDriveValue);
        LeftFrontDrive.setPower(leftFrontDriveValue);
        LeftRearDrive.setPower(leftRearDriveValue);

        if(LiftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
        {
            LiftMotor.setPower(liftMotorValue);
        }

        RightGripperServo.setPosition(rightGripperPosition);
        LeftGripperServo.setPosition(leftGripperPosition);
    }
}
