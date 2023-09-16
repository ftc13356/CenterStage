package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import android.annotation.SuppressLint;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class RFLogger {
    public static Logger LOGGER;
    ArrayList<FileHandler> handlerList = new ArrayList<>();
    SimpleFormatter sh = new SimpleFormatter();
    FileHandler fh;
    Level logLevel = Level.ALL;
    static FileHandler GeneralFH, AutonomousFH, HardwareFH, QueuerFH;

    public enum Files {
        GENERAL_LOG("/sdcard/tmp/General.log", 0),
        AUTONOMOUS_LOG("/sdcard/tmp/Auton.log", 1),
        HARDWARE_LOG("/sdcard/tmp/Hardware.log", 2),
        QUEUER_LOG("/sdcard/tmp/Queuer.log", 3);

        String filePath;
        int index;

        Files(String p_filePath, int p_index){
            filePath = p_filePath;
            index = p_index;
        }
    }

    public enum Severity {
        ALL(Level.ALL),
        SEVERE(Level.SEVERE),
        WARNING(Level.WARNING),
        INFO(Level.INFO),
        CONFIG(Level.CONFIG),
        FINE(Level.FINE),
        FINER(Level.FINER),
        FINEST(Level.FINEST);

        Level logSeverity;

        Severity(Level p_logSeverity){
            logSeverity = p_logSeverity;
        }
    }

    public RFLogger (String className){
        LOGGER = Logger.getLogger(className);
        LOGGER.setLevel(logLevel);
        try {
            GeneralFH = new FileHandler(Files.GENERAL_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            AutonomousFH = new FileHandler(Files.AUTONOMOUS_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            HardwareFH = new FileHandler(Files.HARDWARE_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            QueuerFH = new FileHandler(Files.QUEUER_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        handlerList.add(GeneralFH);
        handlerList.add(AutonomousFH);
        handlerList.add(HardwareFH);
        handlerList.add(QueuerFH);

        GeneralFH.setFormatter(sh);
        AutonomousFH.setFormatter(sh);
        HardwareFH.setFormatter(sh);
        QueuerFH.setFormatter(sh);

        LOGGER.addHandler(GeneralFH);
    }

    public void setLogLevel(Severity p_severity){
        logLevel = p_severity.logSeverity;
        LOGGER.setLevel(logLevel);
    }

    public void log(Severity p_severity, Files p_file, String info){
        fh = handlerList.get(p_file.index);
        LOGGER.addHandler(fh);
        logLevel = p_severity.logSeverity;
        LOGGER.log(logLevel, info);
    }

    public void log(Files p_file, String info){
        fh = handlerList.get(p_file.index);
        LOGGER.addHandler(fh);
        LOGGER.log(logLevel, info);
    }

    public void log(String info){
        fh = handlerList.get(0);
        LOGGER.addHandler(fh);
        LOGGER.log(logLevel, info);
    }
}
