package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class HardwareInterface{
    String deviceName;
    boolean isExpansionHub;
    int port;

    public HardwareInterface(String deviceName, boolean isExpansionHub, int port) {
        this.deviceName = deviceName;
        this.isExpansionHub = isExpansionHub;
        this.port = port;
    }

    public String getDeviceName() {
        return deviceName;
    }

    public boolean isExpansionHub() {
        return isExpansionHub;
    }

    public int getPort() {
        return port;
    }
}
