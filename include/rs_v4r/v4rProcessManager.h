/**
 * Modified RoboSherlock Process Manager
 * Optimized for the Suturo Perception Pipelines
 * @author  Vanessa Hassouna (hassouna@uni-bremen.de)
 */
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/flowcontrol/RSProcessManager.h>
#include <rs_v4r/suturo_conversion.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>

using namespace suturo_perception_msgs;

class v4rProcessManager {
private:
    std::string savePath;
    ros::NodeHandle nh;
    std::string name;
    bool visualize = false;
    bool filter_regions = false;
    std::vector<std::string> regions;
    void getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data);
public:
    // Public Parameters
    RSAnalysisEngine engine;
    rs::Visualizer visualizer;

    // ROS stuff for image topics
    ros::ServiceServer vis_service;
    image_transport::Publisher image_pub;
    image_transport::ImageTransport image_transport;

    // Functions
    v4rProcessManager(ros::NodeHandle n, std::string &name);
    v4rProcessManager(ros::NodeHandle n, std::string savePath, std::string &name);
    ~v4rProcessManager(){};

    void setup();

    void init(std::string &pipeline);

    void run(std::map<std::string, boost::any> args, std::vector<ObjectDetectionData> &detectionData);

    /**
     * Determines if a vertical plane is visible
     * Necessary for door detection
     * @return
     */
    bool has_vertical_plane();

    bool visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
                            robosherlock_msgs::RSVisControl::Response &res);
};
