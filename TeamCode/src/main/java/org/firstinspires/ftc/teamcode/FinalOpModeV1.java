package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.SwitchableLight;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="FinalOpModeV1", group="Tele-Op")

public class FinalOpModeV1 extends LinearOpMode{
    //Initializes the robot hardware variables
    HardwareOmni robot = new HardwareOmni();
    public void runOpMode() {
        //Retrieves the mappings from runtime
        robot.init(hardwareMap);
        double power = 0.2;

        //Control variables
        //Light switch
        boolean bPrevState = false;
        boolean bCurrState = bPrevState;
        //Whipper toggle one way
        boolean dUpPrevState = false;
        boolean dUpCurrState = dUpPrevState;
        //Whipper toggle the other way
        boolean dDownPrevState = false;
        boolean dDownCurrState = dDownPrevState;

        //Arm
        boolean dPadUp;
        boolean dPadDown;
        //Scoop
        boolean dPadRight;
        boolean dPadLeft;

        //Drive train vector
        double left_y, left_x;
        //Rotation
        double left_t, right_t;
        //Rotated drive train vectors
        double abs_x, abs_y;

        //Extension control
        double right_y, right_x;
        //Extension time limiters
        double horizontalTime, verticalTime;

        //Sensor variables
        //Gyroscope angle
        double g_angle;



        //Telemetry initialized message
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //Gyro calibrating
        robot.modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        //Finalized gyro calibration.
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        //Wait until op-mode start
        waitForStart();
        //Resets timer
        robot.timer.reset();
        //While loop for robot operation
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.servo2.setPosition(0.11);
            } else {
                robot.servo2.setPosition(0.43);
            }

            if (gamepad1.right_stick_button) {
                robot.servo1.setPosition(0);
            } else {
                robot.servo1.setPosition(1);
            }
            //Light switch control for color sensor
            bCurrState = gamepad1.b;
            //Checks for a different state
            if (bCurrState != bPrevState) {
                //Checks if the button is enabled
                if (bCurrState) {
                    //Checks if the robot has a switchable light
                    if (robot.colorSensor instanceof SwitchableLight) {
                        //Toggles light
                        SwitchableLight light = (SwitchableLight) robot.colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            //Updates bPrevState
            bPrevState = bCurrState;

            //Whipper switch
            dUpCurrState = gamepad1.dpad_up;
            dDownCurrState = gamepad1.dpad_down;

            if (dUpCurrState != dUpPrevState || dDownCurrState != dDownPrevState) {
                if (dUpCurrState && !dDownCurrState) {
                    robot.servo3.setPower(-1);
                } else if (dDownCurrState && !dUpCurrState) {
                    robot.servo3.setPower(1);
                } else if (!dUpCurrState && !dDownCurrState) {
                    robot.servo3.setPower(0);
                }
            }
            //Updates dDownPrevState
            dDownPrevState = dDownCurrState;
            //Updates xPrevState
            dUpPrevState = dUpCurrState;

            //Lift
            robot.motor6.setPower(gamepad1.right_stick_y);

            //Extension
            robot.motor5.setPower(-gamepad1.right_stick_x);

            //Bucket Flipper
            if (gamepad1.dpad_right) {
                telemetry.addLine()
                        .addData("Bucket Flipping Back:", 0.4);
                robot.motor8.setPower(0.15);
            } else if(gamepad1.dpad_left) {
                telemetry.addLine()
                        .addData("Bucket Returning:", 0.3);
                robot.motor8.setPower(-0.15);
            } else {
                robot.motor8.setPower(0);
            }

            //Arm
            if (gamepad1.x) {
                robot.motor7.setPower(0.45);
            } else if (gamepad1.y) {
                robot.motor7.setPower(-0.45);
            } else {
                robot.motor7.setPower(0);
            }

            //Gamepad's left stick x and y values
            //Inverts y's sign
            left_y = -gamepad1.left_stick_y;
            //Receives x
            left_x = gamepad1.left_stick_x;

            //Gamepad's left and right trigger values
            left_t = gamepad1.left_trigger;
            right_t = gamepad1.right_trigger;

            //Robot Heading Unit Vector

            //Boolean for distance reset
            g_angle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addLine()
                    .addData("Angle:", g_angle);
            g_angle *= Math.PI / 180;
            abs_x = (left_x * Math.cos(-g_angle) - left_y * Math.sin(-g_angle));
            abs_y = (left_x * Math.sin(-g_angle) + left_y * Math.cos(-g_angle));

            telemetry.addLine()
                    .addData("Left Stick X:", left_x)
                    .addData("Left Stick Y:", left_y);
            //Power variable (0,1), average drive train motor speed

            //x component vector
            //motor 2
            robot.motor2.setPower(power * (-abs_x + left_t - right_t));
            //motor4
            robot.motor4.setPower(power * (abs_x + left_t - right_t));

            //y vector
            //motor1
            robot.motor1.setPower(power * (abs_y + left_t - right_t));
            //motor3
            robot.motor3.setPower(power * (-abs_y + left_t - right_t));
            telemetry.update();
        }
    }
}
