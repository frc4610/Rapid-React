package frc.robot.utils.CAN;

import org.json.simple.JSONObject;

public class CANDevice {
  public final double bootloaderRev;
  public final double currentVers;
  public final int dynId;
  public final double hardwareRev;
  public final int id;
  public final String manufacturerData;
  public final String model;
  public final String name;
  public final String softwareStatus;
  public final int uniqID;
  public final String vendor;

  public CANDevice(JSONObject deviceFromArray) {
    bootloaderRev = Double.parseDouble((String) deviceFromArray.get("BootloaderRev"));
    currentVers = Double.parseDouble((String) deviceFromArray.get("CurrentVers"));
    dynId = (int) (long) deviceFromArray.get("DynID");
    hardwareRev = Double.parseDouble((String) deviceFromArray.get("HardwareRev"));
    id = (int) (long) deviceFromArray.get("ID");
    manufacturerData = (String) deviceFromArray.get("ManDate");
    model = (String) deviceFromArray.get("Model");
    name = (String) deviceFromArray.get("Name");
    softwareStatus = (String) deviceFromArray.get("SoftStatus");
    uniqID = (int) (long) deviceFromArray.get("UniqID");
    vendor = (String) deviceFromArray.get("Vendor");
    System.out.println("Found " + name + " CAN ID: " + uniqID);
  }
}
