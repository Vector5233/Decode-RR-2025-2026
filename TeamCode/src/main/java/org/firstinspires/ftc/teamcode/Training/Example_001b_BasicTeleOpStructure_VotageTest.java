package org.firstinspires.ftc.teamcode.Training;
// Cover packages (directories), creating a java class, and refactoring
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


// InLine Comments
/*
Block Comments can collapse...
 */


/** Java Doc
 * Configuration file
 *
 */

@Disabled // does not show up on DriverStation- must be commented out before uploading
@TeleOp(group = "Primary", name = "VoltageTest")
public class Example_001b_BasicTeleOpStructure_VotageTest extends LinearOpMode {
    //public - An access modifier indicating this method can be called from outside this class
    //class - A collection/blueprint of members/fields/global variables and methods and functions
    //Classes are named in UpperCamelCase
    //extends (inheritance) - This class 'inherits' or 'is the child of' LinearOpMode
    //LinearOpMode - linear logic
    //Ctrl-Click on LinearOpMode to view the class

    /**
     * Some teams us interative "OpMode"  in telop for state machine changes.
     * init() - Required - runs when Init is pressed on Driver Station
     * init_loop() - optional Runs repeatedly after INIT, but before play is pressed on the Driver Station.
     * start() - Optional runs once after PLAY is pressed on the Driver station
     * loop() Required - runs repeatedly after PLAY is pressed until STOP is pressed on the Driver Station
     * stop()- optional - runs once after STOP is pressed on the Driver Station.
     * */

    //Define Global Variable/Members/Fields
    //Think of it as creating a bucket that we put things in.
    //Global Variables are named in lowerCamelCase
    //int, double, boolean are primitive types common in FTC.

    private VoltageSensor batteryVoltageSensor;

    @Override // We are replacing the runOpMode methond in the parent class


    public void  runOpMode() throws InterruptedException{
    // void - This method does not return an information (It doesn't talk back)
    //Method are named in lowerCamelCase.
        //throws InterruptedException - optional - Need if we use sleep for delay/ pauses in the code(not recommended).
        //{}- Code blocks -Code to be executed.
        //";" are used to end a statement/sentence.

        // This is what happens when you press init on the Driver Station... Things to run once.
        initHardware();

        while(!isStarted()){}
        // while waiting to start(play), do this...

        sensorTelemetry();
        waitForStart();// Pressing play
        //Does anything need to happen before the while loop such starting a timer or turning of something memory intensive?

        while(opModeIsActive()){// or !isStopRequested()
        //What needs to happem over and over again
            sensorTelemetry();
        }

    }
    public void initHardware(){
        //Method stacking
        //call our methods for all hardware such as DC motors and Servos
        initVoltageSensor();
    }
    public void initVoltageSensor(){// This is a nested/Child code block
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void sensorTelemetry(){
        telemetry.addData("getVoltage", batteryVoltageSensor.getVoltage());
        telemetry.update();

    }



}
