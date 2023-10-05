package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fault.FaultGrabber;
import frc.robot.Constants.RobotMap;

public class FaultMonitor extends SubsystemBase {
    public PowerDistribution pdh;
    private ShuffleboardTab tab;

    /**
     * Grabs faults from hardware not used in other subsystems
     */
    public FaultMonitor() {
        this.tab = Shuffleboard.getTab("faults");
        this.pdh = new PowerDistribution(RobotMap.CAN.PDH, ModuleType.kRev);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        tab.addBooleanArray("PDH Current Faults", () -> FaultGrabber.grabPDH(pdh).currentToBooleanArray());
        tab.addBooleanArray("PDH Current Faults", () -> FaultGrabber.grabPDH(pdh).stickyToBooleanArray());
        tab.addBooleanArray("Rio Current Faults", () -> FaultGrabber.grabRio().currentToBooleanArray());
        tab.addBooleanArray("PDH Current Faults", () -> FaultGrabber.grabRio().stickyToBooleanArray());
        tab.addBoolean("Has Faults", () -> !(FaultGrabber.grabPDH(pdh).hasFaults() || FaultGrabber.grabRio().hasFaults()));
        tab.addBoolean("Has Sticky Faults", () -> !(FaultGrabber.grabPDH(pdh).hasStickyFaults() || FaultGrabber.grabRio().hasStickyFaults()));
    }
}
