package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


/**
 * Created by agb on 11/27/2017.
 */

@TeleOp(name = "Servo Position Finder")

public class ServoPositionFinder extends OpMode
{

    Servo LeftGripperServo;
    Servo RightGripperServo;
    Servo JewelArmServo;

    ServoController ServoController1;

    @Override
    public void init()
    {
        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        ServoController1 = hardwareMap.servoController.get("servo controller 1");
        JewelArmServo = hardwareMap.servo.get("jewel arm");
    }

    double leftServoPosition = 0.5;
    double rightServoPosition = 0.5;
    double jewelArmServoPosition = 0.232;
    double downsizingCoefficient = 100;
    boolean isRight = false;

    @Override
    public void loop()
    {
        //if(gamepad1.right_bumper) {isRight = true;}
        //else if(gamepad1.left_bumper) {isRight = false;}

        //if(isRight)
        //{
            //rightServoPosition = rightServoPosition + (gamepad1.right_trigger / downsizingCoefficient) - (gamepad1.left_trigger / downsizingCoefficient);
        //}
        //else
        //{
        //    leftServoPosition = leftServoPosition + (gamepad1.right_trigger / downsizingCoefficient) - (gamepad1.left_trigger / downsizingCoefficient);
        //}

        LeftGripperServo.setPosition(leftServoPosition);
        RightGripperServo.setPosition(rightServoPosition);

        jewelArmServoPosition = jewelArmServoPosition + (gamepad1.right_trigger / downsizingCoefficient) - (gamepad1.left_trigger / downsizingCoefficient);
        JewelArmServo.setPosition(jewelArmServoPosition);
        //telemetry.addData("Left", LeftGripperServo.getPosition());
        //telemetry.addData("Right", RightGripperServo.getPosition());
        telemetry.addData("Arm", JewelArmServo.getPosition());
        telemetry.update();
    }
}
