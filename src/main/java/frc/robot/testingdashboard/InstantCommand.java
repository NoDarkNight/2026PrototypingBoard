// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testingdashboard;

/** Add your docs here. */
public class InstantCommand extends edu.wpi.first.wpilibj2.command.InstantCommand {

    public InstantCommand(SubsystemBase subsystem, String groupName, String name){
        setName(name);
        TestingDashboard.getInstance().registerCommand(subsystem.getName(), groupName, this);
    }

    public InstantCommand(Runnable toRun, SubsystemBase subsystem, String groupName, String name) {
        super(toRun, subsystem);
        setName(name);
        TestingDashboard.getInstance().registerCommand(subsystem.getName(), groupName, this);
    }
}