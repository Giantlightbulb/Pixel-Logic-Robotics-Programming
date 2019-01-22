package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.SwitchableLight;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="FinalOpModeDoubleControlV1", group="Tele-Op")

public class FinalOpModeDoubleControlV1 extends LinearOpMode{
    //Initializes the robot hardware variables
    HardwareOmni robot = new HardwareOmni();
    public void runOpMode() {
        //Retrieves the mappings from runtime
        robot.init(hardwareMap);
        double power = 0.3;

        //Control variables
        //Light switch
        boolean bPrevState = false;
        boolean bCurrState = bPrevState;
        //Whipper toggle one way
        boolean dUpPrevState = false;
        boolean dUpCurrState = dUpPrevState;
        //Whipper toggle the other way


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
        //double servoPosition = 0.0;
        //Wait until op-mode start
        waitForStart();
        //Resets timer
        robot.timer.reset();
        robot.servo2.setPosition(1.0);
        //While loop for robot operation
        while (opModeIsActive()) {

            //Bucket Flipper
            if (gamepad1.right_bumper) {
                telemetry.addLine()
                        .addData("Bucket Flipping Back:", 0.4);
                robot.motor8.setPower(-0.15);
            } else if(gamepad1.left_bumper) {
                telemetry.addLine()
                        .addData("Bucket Returning:", 0.3);
                robot.motor8.setPower(0.15);
            } else {
                robot.motor8.setPower(0);
            }

            if (gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y) {
                robot.servo1.setPosition(0);
            } else {
                robot.servo1.setPosition(1);
            }

            if (gamepad2.x) {
                robot.servo2.setPosition(0);
            } else if(gamepad2.y){
                robot.servo2.setPosition(1);
            }

            /*
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
            */

            //Whipper switch


            if (gamepad2.left_bumper) {
                    robot.servo3.setPower(-1); //vacuum
                } else if (gamepad2.right_bumper) {
                    robot.servo3.setPower(1); //spit
                } else{
                    robot.servo3.setPower(0);
                }



            //Horizontal Extension
            robot.motor5.setPower(0.8*gamepad2.right_stick_y);

            //Vertical Extension
            robot.motor6.setPower(0.8*gamepad2.left_stick_y);


            //Chariot Lift
            robot.motor7.setPower(0.8*gamepad1.right_stick_y);


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


