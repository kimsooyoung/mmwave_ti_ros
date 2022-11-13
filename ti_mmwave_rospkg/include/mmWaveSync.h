#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdlib>
#include <fstream>
#include <regex>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <thread>
#include <signal.h>

#include <GPIO.h>
#include <serial/serial.h>
#include "ParameterParser.h"
#include "ti_mmwave_rospkg/mmWaveCLI.h"

class Sync {
    public:
        Sync (ros::NodeHandle, ros::NodeHandle);
        ~Sync() {};

        virtual void start () {configureSensors();};

        /** Boolean to indicate if the node should terminate */
        bool m_done;

    private:
        /** Function which uses the mmWaveCLI service to configure mmWave sensors */
        void configureSensors();

        /** Synchronization type: HW, SW, or none */
        std::string m_syncType;

        /** Path to .cfg file used to configure sensors */
        std::string m_configFile;
    
    protected:
        ros::NodeHandle m_nh;

        /** Total number of sensors */
        int32_t m_numSensors;
        
        /** Duration to wait between sending start command to subsequent sensors */
        float m_delay;
};

/* SW based synchronization object. */ 
class SWSync : public Sync {
    public:     
        SWSync (ros::NodeHandle, ros::NodeHandle);
        ~SWSync();

        void start () override { startSyncSW(); };

    private:
        /** Contains pointers to serial object for each sensor */
        std::vector<serial::Serial*> m_serialObjects;

        /** SW based sync start function. */
        void startSyncSW();
};

/* HW based synchronization object. */
class HWSync : public Sync {
    public:
        HWSync(ros::NodeHandle, ros::NodeHandle);
        ~HWSync() {};

        void start () override { startSyncHW(); };

    private:
        /** Pin on SK-TDA4VM 40-pin header for generating triggering pulse */
        int32_t m_pwmPin;

        /** Duty cycle percentage for the triggering pulse */
        float m_dutyCycle;

        /** Frequency in Hz for the triggering pulse */
        int32_t m_pulseFreq;

        std::map<std::string, int32_t> m_outputPins;

        int32_t get_output_pin();

        /** HW based sync start function. */
        void startSyncHW();
};