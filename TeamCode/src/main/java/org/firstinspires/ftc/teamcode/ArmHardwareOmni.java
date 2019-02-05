//HARDWARE OMNI

package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Sensors
//   Gyro
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//  ODS
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//Touch Sensor
import com.qualcomm.robotcore.hardware.TouchSensor;
//  Color Sensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

//Range Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//Compass Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

//Android App Control
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


public class ArmHardwareOmni {
    /* local OpMode members. */
    private HardwareMap hwMap = null;
    public ElapsedTime timer = new ElapsedTime();

    //Constants
    public final double basePosition = 0.25;
    public final double setPosition = 0.7;

    //Base rotation
    public double rotation = 0;

    //Initializes hardware variables
    //Drive train motors
    public DcMotor frontDrive; //
    public DcMotor backDrive; //
    public DcMotor leftDrive; //
    public DcMotor rightDrive; //

    public DcMotor forklift; //Articulating Arm
    //Telescoping lift motor
    public DcMotor verticalLift; //Vertical Extension

    //Chariot Motors
    //Arm
    public DcMotor leftExtension; //Left Arm Extender
    //Back Flip
    public DcMotor rightExtension; //Right Arm Extender

    //Servos
    //Mascot
    public Servo mascot;
    //Replacement CR
    public Servo fakeCR;
    //Whipper
    public CRServo vacuum;
    public CRServo innerVac;

    //Initializes Sensors
    //  Gyro
    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //  Touch Sensor
    public TouchSensor touch;

    //Axes
    NormalizedRGBA colors = new NormalizedRGBA();
    //  ODS
    public OpticalDistanceSensor ods;
    //  Color Sensor
    public NormalizedColorSensor colorSensor;
    //  Compass Sensors
    public ModernRoboticsI2cCompassSensor compass1;
    public ModernRoboticsI2cCompassSensor compass2;

    //  Range Sensor
    public ModernRoboticsI2cRangeSensor rangeSensor;
    //  Initializes layout
    public View relativeLayout;

    public Telemetry localTelemetry;
    
    public LinearOpMode linearOpMode;

    //Constructor (Currently empty)
    public ArmHardwareOmni() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode linearOpMode, HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.linearOpMode = linearOpMode;
        hwMap = ahwMap;
        localTelemetry = telemetry;

        //Hardware definitions
        //Drivetrain motors
        frontDrive = hwMap.get(DcMotor.class, "frontDrive");
        frontDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive = hwMap.get(DcMotor.class, "backDrive");
        backDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //Articulating Arm
        forklift = hwMap.get(DcMotor.class, "forklift");
        //Telescoping lift motor
        verticalLift = hwMap.get(DcMotor.class, "verticalLift");
        //Tape Measure Extensions
        leftExtension = hwMap.get(DcMotor.class, "leftExtension");
        rightExtension = hwMap.get(DcMotor.class, "rightExtension");

        //Servos
        //servo1 = Mascot Dump
        mascot = hwMap.get(Servo.class, "mascot");
        //servo2 = Robot Lock
        fakeCR = hwMap.get(Servo.class, "fakeCR");
        //servo3 = Vacuum Whipper
        vacuum = hwMap.get(CRServo.class, "vacuum");
        //servo4 = Inner Vacuum Whipper
        innerVac = hwMap.get(CRServo.class, "innerVacuum");

        //  Sensors
        //  Touch Sensor
        touch = hwMap.get(ModernRoboticsTouchSensor.class, "touch");
        //  Gyro
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        //  ODS
        ods = hwMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        //  Color Sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        //      Detects switchable light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        //  Compasses
        compass1 = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass1");
        compass2 = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass2");
        //  Range Sensor
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //  Application Color
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        //  Set all motors to zero power
        frontDrive.setPower(0);
        backDrive.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        forklift.setPower(0);
        verticalLift.setPower(0);
        leftExtension.setPower(0);
        rightExtension.setPower(0);
    }

    public void runMotor(DcMotor motor, double power, double time, String label) {
        timer.reset();
        motor.setPower(power);
        while (linearOpMode.opModeIsActive() && (timer.seconds() < time)) {
            localTelemetry.addData(label, "%2.5f S Elapsed", timer.seconds());
            localTelemetry.update();
        }
        motor.setPower(0);
        timer.reset();
    }

    public void runDriveTrain(DcMotor motor1, DcMotor motor2, double power, double time, String label) {
        timer.reset();
        motor1.setPower(power);
        motor2.setPower(power);
        while (linearOpMode.opModeIsActive() && (timer.seconds() < time)) {
            localTelemetry.addData(label, "%2.5f S Elapsed", timer.seconds());
            localTelemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(0);
        timer.reset();
    }

    public float getAngle() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void rotateTheta(double time, double theta, String label) {
        //Rotates to a certain angle
        rotation += theta;
        if (rotation > 180) {
            rotation %= 180;
            rotation -= 180;
        } else if (rotation < -180) {
            rotation %= 180;
            rotation += 180;
        }
        timer.reset();
        while (linearOpMode.opModeIsActive() && (timer.seconds() < time) && getAngle() != rotation) {
            double power = getRotationPower(rotation);
            frontDrive.setPower(-power);
            backDrive.setPower(power);
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
            localTelemetry.addData(label, "%2.5f S Elapsed", timer.seconds());
            localTelemetry.addData("angle", "%s deg", formatFloat(getAngle()));
            localTelemetry.update();
        }
        frontDrive.setPower(0);
        backDrive.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        timer.reset();
    }

    public double getRotationPower(double theta) {
        double angle = theta - getAngle();
        double power;
        power = 0.25 * Math.cbrt(angle/180);
        localTelemetry.addLine()
                .addData("Power", power);
        return power;
    }

    public boolean checkColor() {
        colors = colorSensor.getNormalizedColors();
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        int color = colors.toColor();
        int alpha = Color.alpha(color);
        int red = Color.red(color);
        int green = Color.green(color);
        int blue = Color.blue(color);
        return ((Math.abs(alpha - 70) < 75 ) &&
                (Math.abs(red - 255) < 75 ) &&
                (Math.abs(green - 107) < 75 ) &&
                (Math.abs(blue) < 30 ));
    }
    public void DriveForwardDistance(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut){

        // Reset encoders
        firstMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Target Position b
        firstMotor.setTargetPosition(distance);
        secondMotor.setTargetPosition(distance);

        // Set to RUN_TO_POSITION mode
        firstMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secondMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Drive Power
        firstMotor.setPower(power);
        secondMotor.setPower(power);
        timer.reset();

        while((linearOpMode.opModeIsActive())&&(firstMotor.isBusy() || secondMotor.isBusy())){
            // wait
            localTelemetry.addData("Path1",  "Going to %7d ,  currently at %7d and %7d.", firstMotor.getTargetPosition(),  firstMotor.getCurrentPosition(), secondMotor.getCurrentPosition());
            localTelemetry.update();
        }

        firstMotor.setPower(0);
        secondMotor.setPower(0);

        firstMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
}
