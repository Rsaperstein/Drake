#pragma once

#include <frc/WPILib.h>
#include <MicroLidar.h>
#include <LineSensor.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"


class DalekShuffleboard
{
public:
    DalekShuffleboard(MicroLidar *microLidar, LineSensor *lineSensor);
    ~DalekShuffleboard();
    
    bool calibrateLidar;
    int resetLidar[LIDAR_COUNT];
    
    void init();
    void continious();
private:
    MicroLidar *microLidar;
    LineSensor *lineSensor;
};

