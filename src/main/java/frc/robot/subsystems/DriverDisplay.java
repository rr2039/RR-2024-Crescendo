
package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverDisplay extends SubsystemBase {

    //EXAMPLE:
    //public static ShuffleboardTab arm = Shuffleboard.getTab("Arm"); CREATING tab

    //DriverDisplay.armTarget.setDouble(a);                                       THIS is how to update shuffleboard
    //public static GenericEntry armTarget = arm.add("Arm Target", 0).getEntry(); THIS is how to creat shuffle board

    //selectedAutoSequence = (int)DriverDisplay.AutoSequence.getInteger(Constants.defaultAutoSequence); GET DATA from shuffle board
    //isRedAlliance = DriverStation.getAlliance().toString().equals("Optional[Red]"); GET DATA from driver station

    //flipper
    public static ShuffleboardTab flipperTab = Shuffleboard.getTab("Flipper");
    public static GenericEntry flipperPos = flipperTab.add("FlipperPos", 0).getEntry();
    public static GenericEntry flipperSetpoint =  flipperTab.add("FlipperSetpoint", 0).getEntry();
    public static GenericEntry appliedOutput = flipperTab.add("AppliedOutput", 0).getEntry();
    public static GenericEntry busVoltage = flipperTab.add("BusVoltage", 0).getEntry();

    
    
}
