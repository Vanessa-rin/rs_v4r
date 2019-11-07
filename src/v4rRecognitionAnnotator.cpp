/**
 * v4r Annotator that uses external code to get grasp POSES
 * @author Vanessa Hassouna (hassouna@uni-bremen.de)
 */

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
//PCL
#include "uima/api.hpp"

#include <rs/scene_cas.h>
#include <rs/utils/exception.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>


#include <termios.h>
#include <signal.h>
#include <sys/stat.h>

//PCL
#include <pcl-1.7/pcl/impl/pcl_base.hpp>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>


using namespace uima;

class v4rRecognitionAnnotator : public Annotator {
private:


public:

    TyErrorId initialize(AnnotatorContext &ctx) {
      outInfo("initialize");
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy() {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec) {
      outInfo("processing");

      rs::SceneCas cas(tcas);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
      rs::StopWatch clock;
      cas.get(VIEW_CLOUD, *cloud_ptr);

      rs::Scene scene = cas.getScene();
      std::vector<rs::ObjectHypothesis> clusters;
      scene.identifiables.filter(clusters);
      for (auto &cluster : clusters) {

        //pointcloud filtert with indices of cluster.
        if (cluster.points.has()) {
          pcl::PointIndicesPtr indices(new pcl::PointIndices());
          rs::conversion::from(((rs::ReferenceClusterPoints) cluster.points.get()).indices.get(), *indices);
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

          pcl::ExtractIndices < pcl::PointXYZRGBA > ei;
          ei.setInputCloud(cloud_ptr);
          ei.setIndices(indices);
          ei.filter(*cluster_cloud);

          //TODO ->make msg, send server goal, recieve msgs -> annotate

        }
      }
        return UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(v4rRecognitionAnnotator)