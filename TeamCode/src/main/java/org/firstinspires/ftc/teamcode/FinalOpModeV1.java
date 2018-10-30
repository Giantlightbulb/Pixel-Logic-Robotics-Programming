package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name="FinalOpModeV1", group="Tele-Op")

public class FinalOpModeV1 extends LinearOpMode{
    //Initializes the robot hardware variables
    HardwareOmni robot = new HardwareOmni();
    public void runOpMode() {
        //Retrieves the mappings from runtime
        robot.init(hardwareMap);
        double power = 0.2;

        //Variables for switchable light
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean aButton;
        boolean dPadUp;
        boolean dPadDown;


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


        //Variable instantiation
        double left_y, left_x;
        double left_t, right_t;
        double g_angle;
        double abs_x, abs_y;
        int step = 1;

        //Wait until phone interrupt
        waitForStart();
        robot.timer.reset();

        //While loop for robot operation
        while (opModeIsActive()) {
            bCurrState = gamepad1.x;
            aButton = gamepad1.a;


            if (bCurrState != bPrevState) {
                if (bCurrState) {
                    if (robot.colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight) robot.colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }

            robot.motor5.setPower(gamepad1.right_stick_y);
            bPrevState = bCurrState;

            robot.colors = robot.colorSensor.getNormalizedColors();

            //Gamepad's left stick x and y values
            left_y = -gamepad1.left_stick_y;
            left_x = gamepad1.left_stick_x;

            //Gamepad's left and right trigger values
            left_t = gamepad1.left_trigger;
            right_t = gamepad1.right_trigger;

            //Robot Heading Unit Vector

            //Boolean for distance reset
            g_angle = robot.gyro.getAngularOrientation(robot.aRefInt, robot.aOrderXYZ, robot.aUnit).firstAngle;
            g_angle *= Math.PI / 180;
            abs_x = (left_x * Math.cos(-g_angle) - left_y * Math.sin(-g_angle));
            abs_y = (left_x * Math.sin(-g_angle) + left_y * Math.cos(-g_angle));

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
        }
    }
}
