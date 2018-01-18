package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by agb on 1/5/2018.
 */
@TeleOp(name = "lift encoder output")
public class encoderTelemetry extends OpMode {

    DcMotor LiftMotor;

    Servo LeftGripperServo;
    Servo RightGripperServo;
    Servo JewelArmServo;

    ServoController ServoController1;

    @Override
    public void init() {
        double leftGripperHoldingOffsetFromHalf = 0.030; // smaller number is closed tighter
        double rightGripperHoldingOffsetFromHalf = 0.001; // larger number is closed tighter
        double jewelArmServoRestPosition = 0.19;
        double gripperHoldingSqueeze = 0.035;

        LiftMotor = hardwareMap.dcMotor.get("lift motor");

        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        JewelArmServo = hardwareMap.servo.get("jewel arm");
        ServoController1 = hardwareMap.servoController.get("servo controller 1");

        LeftGripperServo.setPosition(0.5 + leftGripperHoldingOffsetFromHalf - gripperHoldingSqueeze);
        RightGripperServo.setPosition(0.5 + rightGripperHoldingOffsetFromHalf + gripperHoldingSqueeze);
        JewelArmServo.setPosition(jewelArmServoRestPosition);
    }

    @Override
    public void loop() {
        LiftMotor.setPower((gamepad2.right_trigger * gamepad2.right_trigger) - (gamepad2.left_trigger * gamepad2.left_trigger));
        telemetry.addData("Lift Encoder Value:", LiftMotor.getCurrentPosition());
    }
}
