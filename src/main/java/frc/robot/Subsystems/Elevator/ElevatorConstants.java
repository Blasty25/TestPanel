// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.elevator;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;

/** Add your docs here. */
public class ElevatorConstants {

    //sim stuff
    public static final double elevatorMass = Pounds.of(30.0).in(Kilograms);  //Returns The mass in kilograms
    public static final double drumRadius = Inches.of(0.9175).in(Meters);
    public static final double gearing = 0.875; //Check with turbo
    public static final double maxHeight = Inches.of(44).in(Meters);
    public static final double minHeight = Inches.of(0).in(Meters);
    public static final boolean simGravity = false;
    public static final DCMotor gearbox = DCMotor.getNEO(2);

    //funky isfinished stuff
    public static final double tolerance = 0.01;

    //HARDWARE!!!!
    public static final int leftID = 31;
    public static final int rightID = 32;
    public static final double positionConversionFactor = 0.01511; //Smth like that check with turbo



    //Dw bout this i use strings to set my setpoints
    public static final String stow = "STOW";
    public static final String l1 = "L1";
    public static final String l2 = "L2";
    public static final String l3 = "L3";
    public static final String l4 = "L4";


    //TARGET SETPOINTS in INCHES!!! 
    public static final double L4 = 40.0;
    public static final double L3 = 34.0;
    public static final double L2 = 20.0;
    public static final double L1 = 12.0;
    public static final double STOW = 7.0;
}
