#include "lio_sam/cloud_info.h"
#include "lio_sam_utility.h"

struct smoothness_t {
  float value;  // 曲率值
  size_t ind;   // 激光点一维索引
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction : public ParamServer {
 public:
  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  lio_sam::cloud_info cloudInfo;
  std_msgs::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  FeatureExtraction() {
    subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>(
        "lio_sam/deskew/cloud_info", 1,
        &FeatureExtraction::laserCloudInfoHandler, this,
        ros::TransportHints().tcpNoDelay());

    pubLaserCloudInfo =
        nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(
        "lio_sam/feature/cloud_corner", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(
        "lio_sam/feature/cloud_surface", 1);

    initializationValue();
  }

  void initializationValue() {
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
    // 用于面点降采样
    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize,
                               odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel = new int[N_SCAN * Horizon_SCAN];
  }

  void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn) {
    cloudInfo = *msgIn;           // new cloud info
    cloudHeader = msgIn->header;  // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed,
                    *extractedCloud);  // new cloud for extraction

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
  }

  void calculateSmoothness() {
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {
      float diffRange =
          cloudInfo.pointRange[i - 2] + cloudInfo.pointRange[i - 1] -
          cloudInfo.pointRange[i] * 4 + cloudInfo.pointRange[i + 1] +
          cloudInfo.pointRange[i + 2];
      // 距离差值平方作为曲率
      cloudCurvature[i] =
          diffRange *
          diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;
      // 0 表示还未进行特征提取处理,1 表示遮挡、平行，或者已经进行特征提取的点
      cloudNeighborPicked[i] = 0;
      // 1表示角点，-1表示平面点
      cloudLabel[i] = 0;
      // cloudSmoothness for sorting
      // 存储该点曲率值、激光点一维索引
      // 之所以可以这样操作，是因为在initializationValue部分，对cloudSmoothness进行过初始化，
      // 否则直接对cloudSmoothness[i]赋值，一定会报段错误
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i;
    }
  }

  void markOccludedPoints() {
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
      // occluded points
      float depth1 = cloudInfo.pointRange[i];
      float depth2 = cloudInfo.pointRange[i + 1];
      // 两个激光点之间的一维索引差值，如果在一条扫描线上，那么值为1；
      // 如果两个点之间有一些无效点被剔除了，可能会比1大，但不会特别大
      // 如果恰好前一个点在扫描一周的结束时刻，下一个点是另一条扫描线的起始时刻，那么值会很大
      int columnDiff = std::abs(
          int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

      if (columnDiff < 10) {
        // 10 pixel diff in range image
        // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系
        // (也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
        // 远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
        if (depth1 - depth2 > 0.3) {
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        } else if (depth2 - depth1 > 0.3) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
        }
      }
      // parallel beam
      float diff1 = std::abs(
          float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
      float diff2 = std::abs(
          float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));
      // 如果当前点距离左右邻点都过远，则视其为瑕点，因为入射角可能太小导致误差较大
      // 选择距离变化较大的点，并将他们标记为1
      // 情况一： 遇到玻璃
      // 情况二： 遇到空洞
      if (diff1 > 0.1 * cloudInfo.pointRange[i] &&
          diff2 > 0.1 * cloudInfo.pointRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  void extractFeatures() {
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(
        new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++) {
      surfaceCloudScan->clear();

      for (int j = 0; j < 6; j++) {
        // 线性插值
        // 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
        int sp = (cloudInfo.startRingIndex[i] * (6 - j) +
                  cloudInfo.endRingIndex[i] * j) /
                 6;
        int ep = (cloudInfo.startRingIndex[i] * (5 - j) +
                  cloudInfo.endRingIndex[i] * (j + 1)) /
                     6 -
                 1;

        if (sp >= ep) continue;

        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                  by_value());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--) {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] > edgeThreshold) {
            // 当前激光点还未被处理，且曲率大于阈值，则认为是角点
            largestPickedNum++;
            if (largestPickedNum <= 40) {
              cloudLabel[ind] = 1;
              cornerCloud->push_back(extractedCloud->points[ind]);
            } else {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
            for (int l = 1; l <= 5; l++) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10) break;
              cloudNeighborPicked[ind + l] = 1;
            }
            // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
            for (int l = -1; l >= -5; l--) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10) break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++) {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] < surfThreshold) {
            // 当前激光点还未被处理，且曲率小于阈值，则认为是平面点
            cloudLabel[ind] = -1;
            cloudNeighborPicked[ind] = 1;
            // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
            for (int l = 1; l <= 5; l++) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10) break;

              cloudNeighborPicked[ind + l] = 1;
            }
            // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
            for (int l = -1; l >= -5; l--) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10) break;

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++) {
          if (cloudLabel[k] <= 0) {
            surfaceCloudScan->push_back(extractedCloud->points[k]);
          }
        }
      }

      surfaceCloudScanDS->clear();
      downSizeFilter.setInputCloud(surfaceCloudScan);
      downSizeFilter.filter(*surfaceCloudScanDS);

      *surfaceCloud += *surfaceCloudScanDS;
    }
  }

  void freeCloudInfoMemory() {
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
  }

  void publishFeatureCloud() {
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud,
                                          cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud,
                                           cloudHeader.stamp, lidarFrame);
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam");

  FeatureExtraction FE;

  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  return 0;
}
