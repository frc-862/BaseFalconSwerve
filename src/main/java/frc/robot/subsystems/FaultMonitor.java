package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fault.FaultGrabber;
import frc.lib.fault.FaultList;
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

        FaultList pdhFaultList = FaultGrabber.grabPDH(pdh);
        // tab.addBooleanArray("PDH Current Faults", () -> FaultGrabber.grabPDH(pdh).currentToBooleanArray());
        tab.addNumber("PDH Fault Count", () -> FaultGrabber.grabPDH(pdh).getCurrentFaultCount());
        // tab.addBooleanArray("PDH Sticky Faults", () -> FaultGrabber.grabPDH(pdh).stickyToBooleanArray());
        tab.addNumber("PDH Sticky Count", () -> FaultGrabber.grabPDH(pdh).getStickyFaultCount());
        // tab.addString("PDH Current Faults", () -> FaultGrabber.grabPDH(pdh).getCurrentFaults());
        // tab.addString("PDH Sticky Faults", () -> FaultGrabber.grabPDH(pdh).getStickyFaults());
        tab.addBoolean("PDH Faults", () -> pdh.getFaults().CanWarning);
        // tab.addBooleanArray("Rio Current Faults", () -> FaultGrabber.grabRio().currentToBooleanArray());
        // tab.addBooleanArray("Rio Sticky Faults", () -> FaultGrabber.grabRio().stickyToBooleanArray());
        tab.addNumber("Rio Fault Count", () -> FaultGrabber.grabRio().getCurrentFaultCount());
        tab.addNumber("Rio Sticky Count", () -> FaultGrabber.grabRio().getStickyFaultCount());
        tab.addString("Rio Current Faults", () -> FaultGrabber.grabRio().getCurrentFaults());
        tab.addString("Rio Sticky Faults", () -> FaultGrabber.grabRio().getStickyFaults());
        tab.addBoolean("Has Faults", () -> !(FaultGrabber.grabPDH(pdh).hasFaults() || FaultGrabber.grabRio().hasFaults()));
        tab.addBoolean("Has Sticky Faults", () -> !(FaultGrabber.grabPDH(pdh).hasStickyFaults() || FaultGrabber.grabRio().hasStickyFaults()));
    }

    @Override
    public void periodic() {
        // tab.addBooleanArray("PDH Current Faults", () -> FaultGrabber.grabPDH(pdh).currentToBooleanArray());
        // tab.addBooleanArray("PDH Sticky Faults", () -> FaultGrabber.grabPDH(pdh).stickyToBooleanArray());
        // tab.addBooleanArray("Rio Current Faults", () -> FaultGrabber.grabRio().currentToBooleanArray());
        // tab.addBooleanArray("Rio Sticky Faults", () -> FaultGrabber.grabRio().stickyToBooleanArray());
        // tab.addBoolean("Has Faults", () -> !(FaultGrabber.grabPDH(pdh).hasFaults() || FaultGrabber.grabRio().hasFaults()));
        // tab.addBoolean("Has Sticky Faults", () -> !(FaultGrabber.grabPDH(pdh).hasStickyFaults() || FaultGrabber.grabRio().hasStickyFaults()));

        // SmartDashboard.putBooleanArray("PDH Current Faults", FaultGrabber.grabPDH(pdh).currentToBooleanArray());
        // SmartDashboard.putBooleanArray("PDH Current Faults", FaultGrabber.grabPDH(pdh).stickyToBooleanArray());
        // SmartDashboard.putBooleanArray("Rio Current Faults", FaultGrabber.grabRio().currentToBooleanArray());
        // SmartDashboard.putBooleanArray("PDH Current Faults", FaultGrabber.grabRio().stickyToBooleanArray());
        // SmartDashboard.putBoolean("Has Faults", !(FaultGrabber.grabPDH(pdh).hasFaults() || FaultGrabber.grabRio().hasFaults()));
        // SmartDashboard.putBoolean("Has Sticky Faults", !(FaultGrabber.grabPDH(pdh).hasStickyFaults() || FaultGrabber.grabRio().hasStickyFaults()));
    }
}
