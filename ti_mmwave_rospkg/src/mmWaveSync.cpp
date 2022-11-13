#include "mmWaveSync.h"

using namespace std;

Sync *g_syncObjPtr;

void signalHandler(int32_t s)
{
    g_syncObjPtr->m_done = true;
}

Sync::Sync (ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    m_nh = nh;
    m_done = false;
    m_configFile = "";

    private_nh.getParam("trigger_delay", m_delay);
    private_nh.getParam("num_sensors", m_numSensors);
    private_nh.getParam("sync_type", m_syncType);
    private_nh.getParam("config_file", m_configFile);
}

void Sync::configureSensors()
{
    ti_mmwave_rospkg::mmWaveCLI srv;
    if (m_configFile == "")
    {
        ROS_ERROR("mmWaveSync: Missing config file!");
        m_done = true;
        return;
    }

    std::string mmWaveCLIName;
    ros::ServiceClient client;
    ti_mmwave_rospkg::ParameterParser parser;

    for (int32_t i = 0; i < m_numSensors; i++)
    {
        std::ifstream myParams;
        myParams.open(m_configFile);

        mmWaveCLIName = "/mmWaveCLI_" + std::to_string(i);

        //wait for service to become available
        ros::service::waitForService(mmWaveCLIName, 10000);
        client = m_nh.serviceClient<ti_mmwave_rospkg::mmWaveCLI>(mmWaveCLIName);

        if (myParams.is_open())
        {
            while( std::getline(myParams, srv.request.comm)) 
            {
                // Remove Windows carriage-return if present
                srv.request.comm.erase(std::remove(srv.request.comm.begin(), srv.request.comm.end(), '\r'), srv.request.comm.end());
                // Ignore comment lines (first non-space char is '%') or blank lines
                if (!(std::regex_match(srv.request.comm, std::regex("^\\s*%.*")) || std::regex_match(srv.request.comm, std::regex("^\\s*")))) 
                {            
                    if (std::regex_search(srv.request.comm, std::regex("frameCfg")) && m_syncType == "HW")
                    {
                        std::istringstream iss(srv.request.comm);
                        std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                                        std::istream_iterator<std::string>{}};
                        
                        // Set Trigger Delay value in frameCfg
                        tokens[7] = std::to_string(i * m_delay);

                        srv.request.comm = "";
                        for (const auto &piece : tokens) srv.request.comm += (piece + ' ');
                    }

                    if (std::regex_search(srv.request.comm, std::regex("sensorStart")) && m_syncType == "SW")
                    {   
                        // exit without sending sensor start
                        break;
                    }

                    if (client.call(srv)) 
                    {
                        if (std::regex_search(srv.response.resp, std::regex("Done"))) 
                        {  
                            parser.ParamsParser(srv, m_nh);
                        } 
                        else 
                        {
                            ROS_ERROR("mmWaveSync: Command failed (mmWave sensor did not respond with 'Done')");
                            ROS_ERROR("mmWaveSync: Response: '%s'", srv.response.resp.c_str() );
                            m_done = true;
                            return;
                        }
                    } 
                    else 
                    {
                        ROS_ERROR("mmWaveSync: Failed to call service mmWaveCLI");
                        ROS_ERROR("%s", srv.request.comm.c_str() );
                        m_done = true;
                        return;
                    }
                }
            }

            parser.CalParams(m_nh);
            myParams.close();
        }
        else 
        {
            ROS_ERROR("mmWaveSync: Failed to open File %s", m_configFile.c_str());
            m_done = true;
        }   
    }
}

HWSync::HWSync (ros::NodeHandle nh, ros::NodeHandle private_nh) : Sync::Sync(nh, private_nh)
{
    // Populate pulse parameters with user arguments from launch file
    private_nh.getParam("pulse_freq", m_pulseFreq);
    private_nh.getParam("duty_cycle", m_dutyCycle);
    private_nh.getParam("pwm_pin", m_pwmPin);
}

int32_t HWSync::get_output_pin()
{
    if (m_outputPins.find(GPIO::model) == m_outputPins.end())
    {
        ROS_INFO("PWM not supported on this board\n");
        terminate();
    }

    return m_outputPins.at(GPIO::model);
}

void HWSync::startSyncHW()
{
    Sync::start();

    m_outputPins["J721E_SK"] = m_pwmPin;

    // Pin Definitions
    int32_t output_pin = get_output_pin();
    
    // Pin Setup
    // Board pin-numbering scheme
    GPIO::setmode(GPIO::BOARD);

    // set pin as an output pin with optional initial state of LOW
    GPIO::setup(output_pin, GPIO::OUT, GPIO::LOW);
    GPIO::PWM p(output_pin, m_pulseFreq);

    // Enable the PWM
    ROS_INFO("mmWaveSync: Starting HW triggering pulse with frequency: %dhz and duty cycle: %f pct.", m_pulseFreq, m_dutyCycle);
    p.start(m_dutyCycle);

    // Wait for user interrupt via CTRL+C
    while (!m_done)
    {
        ros::Duration(1).sleep();
    }

    // Clean up gpio pins
    p.stop();
    GPIO::cleanup();
}

SWSync::SWSync (ros::NodeHandle nh, ros::NodeHandle private_nh) : Sync::Sync(nh, private_nh)
{    
    m_delay = m_delay / 1000;

    std::vector<serial::Serial*> nulled(m_numSensors);
    m_serialObjects = nulled;

    std::string port_names; 
    private_nh.getParam("serial_ports", port_names);
    ROS_INFO("%s", port_names.c_str());

    std::istringstream iss(port_names);
    std::vector<std::string> ports{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};

    for (int32_t i = 0; i < m_numSensors; i++)
    {
        /* Create Serial port object for each device */
        serial::Serial *mySerialObject = new serial::Serial("", 115200, serial::Timeout::simpleTimeout(1000));
        mySerialObject->setPort(ports[i]);
        m_serialObjects[i] = mySerialObject;
    }
}

SWSync::~SWSync()
{
    for (int32_t i = 0; i < m_numSensors; i++)
    {
        delete m_serialObjects[i];
    }
}

void SWSync::startSyncSW()
{
    Sync::start();

    std::string response;

    for (int32_t i = 0; i < m_numSensors; i++)
    {
        /* Open Serial port and error check */
        try 
        {
            m_serialObjects[i]->open();
        } 
        catch (std::exception &e1) 
        {
            ROS_INFO("mmWaveSync: Failed to open User serial port with error: %s", e1.what());
        }
    }

    /* Send start command to each device */
    ROS_INFO("mmWaveSync: Sending syncronized sensorStart command to devices.");
    for (int32_t i = 0; i < m_numSensors; i++)
    {
        int32_t bytesSent = m_serialObjects[i]->write("sensorStart\n");
        ros::Duration(m_delay).sleep();    
    }

    /* Read response from each device and close serial port */
    for (int32_t i = 0; i < m_numSensors; i++)
    {
        response = "";
        m_serialObjects[i]->readline(response, 1024, ":/>");
        ROS_INFO("mmWaveSync: Received response from sensor %d: '%s'", i, response.c_str());

        m_serialObjects[i]->close();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmWaveSync"); 

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string sync_type;
    private_nh.getParam("sync_type", sync_type);
    if (sync_type == "none")
    {
        return 0;
    }
    else if (sync_type == "SW")
    {
        ROS_INFO("mmWaveSync: SW Synchronization enabled");

        g_syncObjPtr = new SWSync(nh, private_nh);
    }
    else if (sync_type == "HW")
    {
        ROS_INFO("mmWaveSync: HW Synchronization enabled");

        g_syncObjPtr = new HWSync(nh, private_nh);
    }
    else
    {
        ROS_INFO("mmWaveSync: Sync not enabled! 'sync_type' must be 'SW' 'HW' or 'none'");
        return 1;
    }

    // When CTRL+C pressed, signalHandler will be called
    signal(SIGINT, signalHandler);

    g_syncObjPtr->start();

    return 0;
}