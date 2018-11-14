//ABSOLUTE CONTROL COLOR

package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.SwitchableLight;



import android.graphics.Color;



@TeleOp(name="AbsoluteControlColor", group="Basic OP Mode")

public class AbsoluteControlColor extends LinearOpMode{
    //Initializes the robot hardware through the HardwareOmni class
    HardwareOmni robot = new HardwareOmni();

    @Override public void runOpMode() throws InterruptedException {
      // Get a reference to the RelativeLayout so we can later change the background
      // color of the Robot Controller app to match the hue detected by the RGB sensor.
      robot.init(hardwareMap);

      try {
        runSample(); // actually execute the sample
      } finally {
        // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
        // as pure white, but it's too much work to dig out what actually was used, and this is good
        // enough to at least make the screen reasonable again.
        // Set the panel back to the default color
        robot.relativeLayout.post(new Runnable() {
          public void run() {
            robot.relativeLayout.setBackgroundColor(Color.WHITE);
          }
        });
        }
    }

    public void runSample() throws InterruptedException{
        double power = 0.2;


        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean aPrevState = false;
        boolean aCurrState = aPrevState;
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
        double rSum = 0;
        double gSum = 0;
        double bSum = 0;
        int step = 1;
        int colorAvg = 0;

        //Wait until phone interrupt
        waitForStart();
        robot.timer.reset();

        //While loop for robot operation
        while (opModeIsActive()){
            bCurrState = gamepad1.x;



            if (bCurrState != bPrevState) {
              if (bCurrState) {
                if (robot.colorSensor instanceof SwitchableLight) {
                  SwitchableLight light = (SwitchableLight)robot.colorSensor;
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
            g_angle *= Math.PI/180;
            abs_x = (left_x*Math.cos(-g_angle)-left_y*Math.sin(-g_angle));
            abs_y = (left_x*Math.sin(-g_angle)+left_y*Math.cos(-g_angle));

            //Power variable (0,1), average drive train motor speed

            //x component vector
            //motor 2
            robot.motor2.setPower(power*(-abs_x+left_t-right_t));
            //motor4
            robot.motor4.setPower(power*(abs_x+left_t-right_t));

            //y vector
            //motor1
            robot.motor1.setPower(power*(abs_y+left_t-right_t));
            //motor3
            robot.motor3.setPower(power*(-abs_y+left_t-right_t));

            //More telemetry. Adds left stick values and trigger values
            /*
            telemetry.addLine()
                    .addData("right_y", left_y)
                    .addData("left_x", left_x );
            telemetry.addLine()
                    .addData("Motor 1+3", abs_y);
            telemetry.addLine()
                    .addData("Motor 2+4", abs_x);
            telemetry.addLine()
                    .addData("angle", g_angle);
            */
            Color.colorToHSV(robot.colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H",  hsvValues[0])
                    .addData("S",  hsvValues[1])
                    .addData("V",  hsvValues[2]);
            telemetry.addLine()
                    .addData("a",  robot.colors.alpha)
                    .addData("r",  robot.colors.red)
                    .addData("g",  robot.colors.green)
                    .addData("b",  robot.colors.blue);
            int color = robot.colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a",  Color.alpha(color))
                    .addData("r",  Color.red(color))
                    .addData("g",  Color.green(color))
                    .addData("b",  Color.blue(color));
            float max = Math.max(Math.max(Math.max(robot.colors.red, robot.colors.green), robot.colors.blue), robot.colors.alpha);
            robot.colors.red   /= max;
            robot.colors.green /= max;
            robot.colors.blue  /= max;

            color = robot.colors.toColor();
            telemetry.addLine("normalized color:  ")
                            .addData("a",  Color.alpha(color))
                            .addData("r",  Color.red(color))
                            .addData("g",  Color.green(color))
                            .addData("b", Color.blue(color));
            aCurrState = gamepad1.a;
            if (aCurrState != aPrevState) {
                colorAvg += color;
                step++;
            }
            aPrevState = aCurrState;
            telemetry.addLine("Color Average")
                    .addData("a", Color.alpha(colorAvg/step))
                    .addData("r", Color.alpha(colorAvg/step))
                    .addData("g",  Color.green(colorAvg/step))
                    .addData("b", Color.blue(colorAvg/step));

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
            telemetry.addLine()
                    .addData("distance", robot.ods.getRawLightDetected());
            telemetry.addLine().addData("distance normal", robot.ods.getLightDetected());
            telemetry.addLine()
                    .addData("A:", aCurrState);
            //telemetry.addLine().addData("Delta_t", delta_t);

            telemetry.update();

            step ++;

            robot.relativeLayout.post(new Runnable() {
              public void run() {
                robot.relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
              }
            });
        }
    }
}
