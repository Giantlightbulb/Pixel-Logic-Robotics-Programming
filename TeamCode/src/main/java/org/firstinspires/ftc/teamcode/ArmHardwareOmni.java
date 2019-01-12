//HARDWARE OMNI

package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Sensors
//   Gyro
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
//  ODS
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//  Color Sensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

//Range Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//Compass Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Android App Control
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


public class ArmHardwareOmni {
    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime timer = new ElapsedTime();

    //Initializes hardware variables
    //Drive train motors
    public DcMotor frontMotor; //
    public DcMotor backMotor; //
    public DcMotor leftMotor; //
    public DcMotor rightMotor; //

    public DcMotor arm; //Articulating Arm
    //Telescoping lift motor
    public DcMotor verticalLift; //Vertical Extension

    //Chariot Motors
    //Arm
    public DcMotor leftTape; //Left Arm Extender
    //Back Flip
    public DcMotor rightTape; //Right Arm Extender

    //Servos
    //Mascot
    public Servo mascot;
    //Bucket Arm
    public Servo latch;
    //Whipper
    public CRServo vacuum;

    //Initializes Sensors
    //  Gyro
    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //Axes
    public AxesReference aRefInt = AxesReference.INTRINSIC;
    public AxesOrder aOrderXYZ = AxesOrder.XYZ;
    public AngleUnit aUnit = AngleUnit.DEGREES;
    NormalizedRGBA colors = new NormalizedRGBA();
    //  ODS
    public OpticalDistanceSensor ods;
    //  Color Sensor
    NormalizedColorSensor colorSensor;
    //  Compass Sensor
    ModernRoboticsI2cCompassSensor compass;
    //  Range Sensor
    ModernRoboticsI2cRangeSensor rangeSensor;
    //  Initializes layout
    View relativeLayout;

    //Constructor (Currently empty)
    public ArmHardwareOmni() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //Hardware definitions
        //Drivetrain motors
        frontMotor = hwMap.get(DcMotor.class, "frontMotor");
        backMotor = hwMap.get(DcMotor.class, "backMotor");
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        //Articulating Arm
        arm = hwMap.get(DcMotor.class, "arm");
        //Telescoping lift motor
        verticalLift = hwMap.get(DcMotor.class, "verticalLift");
        //Tape Measure Extensions
        leftTape = hwMap.get(DcMotor.class, "leftTape");
        rightTape = hwMap.get(DcMotor.class, "rightTape");

        //Servos
        //servo1 = Mascot Dump
        mascot = hwMap.get(Servo.class, "mascot");
        //servo2 = Robot Lock
        latch = hwMap.get(Servo.class, "latch");
        //servo3 = Vacuum Whipper
        vacuum = hwMap.get(CRServo.class, "vacuum");

        //  Sensors
        //  Gyro
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        //  ODS
        ods = hwMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        //  Color Sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color_sensor");
        //      Detects switchable light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        //  Compass
        compass = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        //  Range Sensor
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        //  Application Color
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        //  Set all motors to zero power
        frontMotor.setPower(0);
        backMotor.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        arm.setPower(0);
        verticalLift.setPower(0);
        leftTape.setPower(0);
        rightTape.setPower(0);

    }
}