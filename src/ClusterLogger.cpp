#include <player_tracker/ClusterLogger.hpp>

namespace logging {

    ClusterLogger::ClusterLogger(ros::NodeHandle &nh, std::string scanTopic, std::string logFilename) : nh(nh), scanTopic(scanTopic), logFilename(logFilename), prevMsg(NULL)
    {
        this->initROSCommunication();
        this->openLogFile();
    }

    ClusterLogger::~ClusterLogger()
    {
        logFile.close();
        windows.clear();
    }


    void ClusterLogger::initROSCommunication()
    {
        std::string clusterTopic;
        ros::NodeHandle privateNH("~");
        privateNH.param<std::string>("cluster_topic", clusterTopic, "detected_leg_clusters");

        laserSubscriber       = nh.subscribe(scanTopic, 1, &ClusterLogger::laserCallback, this);

        laserCloudPublisher   = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
        cloudClusterPublisher = nh.advertise<sensor_msgs::PointCloud2>("clusters", 10);

        assembleScanService   = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    }

    void ClusterLogger::openLogFile()
    {
        // ROS_INFO_STREAM(logFilename);
        logFile.open(logFilename, std::fstream::trunc);
    }

    void ClusterLogger::saveToLogFile(std::string text)
    {
        if (logFile.is_open()) {
            logFile << text.c_str() << std::endl << std::flush;
        }
    }

    void ClusterLogger::closeWindow()
    {
        saveToLogFile("#,#,#,#");
    }


    void ClusterLogger::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (windows.empty()) {
            windows.push_back(WindowReference());
        }
        if (windows.back().currentNum == 10) {
            closeWindow();
            windows.push_back(WindowReference());
        }
        if (!windows.back().valid) {
            fillWindowReference(windows.back(), msg);
        }

        // ROS_INFO_STREAM("time " << windows.back().currentNum);

        if (prevMsg != NULL) {
            // update ending time of window
            windows.back().endTime = prevMsg->header.stamp;
            // publish
            publishWindow(prevMsg);
        }
        windows.back().currentNum++;

        // last message published at next call
        prevMsg = msg;

        while (windows.size() >= 3) {
            windows.pop_front();
        }
    }


    void ClusterLogger::fillWindowReference(WindowReference &window, const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        window.startSequence = msg->header.seq;
        window.startTime = msg->header.stamp;
        window.valid = true;
    }

    void ClusterLogger::publishWindow(sensor_msgs::LaserScan::ConstPtr &msg)
    {
        unsigned int time = windows.back().currentNum;

        sensor_msgs::PointCloud cloud = transformLaserToCloud(msg);

        // set time dimension
        setTimeToCloud(cloud, time);

        // publish point cloud
        laserCloudPublisher.publish(cloud);

        // Compute running window features
        computeSpatialTemporalFeatures(time);
    }

    sensor_msgs::PointCloud ClusterLogger::transformLaserToCloud(const sensor_msgs::LaserScan::ConstPtr &scanMsg)
    {
        sensor_msgs::PointCloud cloud;
        std::string scanFrame = scanMsg->header.frame_id;
        auto time = scanMsg->header.stamp + ros::Duration().fromSec(scanMsg->ranges.size() * scanMsg->time_increment);

        // check whether we can transform data
        try{
            if (!tfListener.waitForTransform(scanFrame, "base_link", time, ros::Duration(1.0))) {
                return cloud;
            }
        }catch (std::exception& ex){
            ROS_ERROR("%s", ex.what());
            return cloud;
        }

        projector.transformLaserScanToPointCloud("base_link", *scanMsg, cloud, tfListener);
        return cloud;
    }

    void ClusterLogger::setTimeToCloud(sensor_msgs::PointCloud &cloud, unsigned int time)
    {
        #pragma omp parallel for
        for (int i = 0; i < cloud.points.size(); i++) {
            cloud.points[i].z = static_cast<double>(time);
        }
    }


    void ClusterLogger::computeSpatialTemporalFeatures(unsigned int time)
    {
        ros::Time now = ros::Time::now();
        // analysis for current clusters and clusters!!!
        PointCloud2List clusters = getClustersInWindow();
        // ROS_INFO_STREAM(time << ": cluster size " << clusters.size());

        if (clusters.empty()) {
            ROS_DEBUG("ON computeSpatialTemporalFeatures: No clusters found. Skipping...");
            return;
        }

        Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;


        FloatList substitutions(clusters.size(), -500);
        for(int i = 0; i < clusters.size(); i++) {
            // ROS_INFO_STREAM("Processing cluster " << i << "/" << clusters.size());

            float angle = extractClusterAngle(clusters[i], eigenvects.col(i));
            IntFloatList similarities = computeSimilarity(clusters[i], clusters);

            std::pair <int,float> maxSim = *std::max_element(similarities.begin(), similarities.end(), compPair);

            if (maxSim.second != 0) {
                substitutions[maxSim.first] = angle;
            } else {
                substitutions.push_back(angle);
            }
        }

        std::stringstream stream;
        for (int i = 0; i < substitutions.size(); i++) {
            if (substitutions[i] == -500) continue;
            stream << now.sec << "." << now.nsec << "," << i << "," << time << "," << substitutions[i] << std::endl; // mancano x ed y
        }

        saveToLogFile(stream.str());
    }

    float ClusterLogger::extractClusterAngle(sensor_msgs::PointCloud2 &cluster, Eigen::MatrixXd::ColXpr eigenCol)
    {
        Eigen::VectorXd eigenvect = computeSmallestEigenvector(cluster);
        eigenCol << eigenvect(0), eigenvect(1), eigenvect(2);
        return computeAngle(eigenvect);
    }

    float ClusterLogger::computeAngle(Eigen::VectorXd &eigenvect)
    {
        // compute dot product between eigenvect and plane_normal
        if (eigenvect(2) > 0) {
            eigenvect = eigenvect * -1;
        }

        Eigen::VectorXd plane_normal(3);
        plane_normal << 0.0, 0.0, 1.0;

        double dot = eigenvect.dot(plane_normal);
        double cosine = dot / (eigenvect.norm() * plane_normal.norm());
        float angle = acos(cosine) * 180.0 / M_PI;
        return angle;
    }


    PointCloud2List ClusterLogger::getClustersInWindow()
    {
        sensor_msgs::PointCloud to_merge_cloud;
        sensor_msgs::PointCloud2 as_pointcloud_2;

        int mergedClouds = assembledCloud(windows.back().startTime, ros::Time::now(), to_merge_cloud);
        // ROS_INFO_STREAM("clouds " << mergedClouds);

        if (!mergedClouds) {
            return PointCloud2List();
        }

        int convertedClouds = sensor_msgs::convertPointCloudToPointCloud2(to_merge_cloud, as_pointcloud_2);
        if (!convertedClouds) {
            return PointCloud2List();
        }

        PointCloud2List clusters = getClusters(as_pointcloud_2);
        return clusters;
    }

    PointCloud2List ClusterLogger::getClusters(sensor_msgs::PointCloud2 &input)
    {
        PointCloud2List clusters;
        pcl::PCLPointCloud2 to_convert;

        // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
        pcl_conversions::toPCL(input, to_convert);

        pcl::PointCloud<pcl::PointXYZ>::Ptr converted(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(to_convert,*converted);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(converted);

        std::vector<pcl::PointIndices> cluster_indices;
        computeClusterIndices(cluster_indices, tree, converted);

        int count = 0;

        #pragma omp parallel for
        for (int i = 0; i < cluster_indices.size(); i++) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); pit++) {
                cloud_cluster->points.push_back(converted->points[*pit]); //*
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Convert the pointcloud to be used in ROS
            sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *output);
            output->header.frame_id = input.header.frame_id;

            // store cluster
            #pragma omp critical
            clusters.push_back(*output);
            count++;
        }

        ROS_DEBUG("Number of clusters: %d", count);
        return clusters;
    }

    Eigen::MatrixXd ClusterLogger::computeClusterVariance(Eigen::MatrixXd &input)
    {
        Eigen::MatrixXd centered = input.colwise() - input.rowwise().mean();
        Eigen::MatrixXd cov = (centered * centered.transpose()) / double(input.cols() - 1);
        return cov;
    }


    Eigen::MatrixXd ClusterLogger::computeClusterVariance(sensor_msgs::PointCloud2 &input)
    {
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            Eigen::MatrixXd point_matrix = cloudPointAsMatrix(input);
            return computeClusterVariance(point_matrix);
        }else{
            ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
        }
    }

    Eigen::MatrixXd ClusterLogger::cloudPointAsMatrix(sensor_msgs::PointCloud2 &input)
    {
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            Eigen::MatrixXd output(3, cloud.points.size());
            #pragma omp parallel for
            for (int i=0; i < cloud.points.size(); i++){
                output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
            }
            return output;
        }else{
            ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
        }
    }

    Eigen::MatrixXd ClusterLogger::cloudPointAsMatrix(sensor_msgs::PointCloud &cloud)
    {
        Eigen::MatrixXd output(3, cloud.points.size());
        #pragma omp parallel for
        for (int i=0; i < cloud.points.size(); i++){
            output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
        }
        return output;
    }

    void ClusterLogger::computeClusterIndices(std::vector<pcl::PointIndices> &cluster_indices, pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree, pcl::PointCloud<pcl::PointXYZ>::Ptr &converted)
    {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.50); // 50cm
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(converted);
        ec.extract(cluster_indices);
    }

    int ClusterLogger::assembledCloud(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud)
    {
        laser_assembler::AssembleScans assembleScanMsg;
        assembleScanMsg.request.begin = from;
        assembleScanMsg.request.end   = to;

        if (assembleScanService.call(assembleScanMsg)){
            ROS_DEBUG("Got cloud with %lu points\n", assembleScanMsg.response.cloud.points.size());
            cloud = assembleScanMsg.response.cloud;
            return 1;
        }else{
            ROS_ERROR("Service call to assemble_scans failed\n");
            return 0;
        }
    }

    IntFloatList ClusterLogger::computeSimilarity(sensor_msgs::PointCloud2 &cluster, PointCloud2List &clusters)
    {
        sensor_msgs::PointCloud clusterConverted;

        if(!sensor_msgs::convertPointCloud2ToPointCloud(cluster, clusterConverted)) {
            ROS_ERROR("Error when converting clusters for comparison.");
        }

        IntFloatList jaccardSim(clusters.size(), {0, -1.0f});

        #pragma omp parallel for
        for(int i = 0; i < clusters.size(); i++) {
            sensor_msgs::PointCloud c;

            sensor_msgs::PointCloud2 ptcloud = clusters[i];

            if(!sensor_msgs::convertPointCloud2ToPointCloud(ptcloud, c)) {
                ROS_ERROR("Error when converting clusters for comparison.");
            }

            float result = computeJaccardSimilarity(c, clusterConverted);
            jaccardSim[i] = IntFloatPair(i, result);
        }

        return jaccardSim;
    }

    float ClusterLogger::computeJaccardSimilarity(sensor_msgs::PointCloud &cloudIn1, sensor_msgs::PointCloud &cloudIn2)
    {
        int qt = 0;

        #pragma omp parallel for reduction(+:qt)
        for(int i=0; i < cloudIn1.points.size(); i++){
            for(int j=0; j < cloudIn2.points.size(); j++){
                if (fabs(cloudIn1.points[i].x - cloudIn2.points[j].x) < 0.001 &&
                        fabs(cloudIn1.points[i].y - cloudIn2.points[j].y) < 0.001 &&
                        fabs(cloudIn1.points[i].z - cloudIn2.points[j].z) < 0.001){
                    qt++;
                }
            }
        }

        int union_size = (cloudIn1.points.size() + cloudIn2.points.size()) - qt;

        return ((float) qt) / ((float)union_size);         // compute jaccard
    }

    Eigen::VectorXd ClusterLogger::computeSmallestEigenvector(Eigen::MatrixXd &cov)
    {
        Eigen::VectorXd eigenvals = cov.eigenvalues().real();
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::MatrixXd eigenvecs = es.eigenvectors().real();
        Eigen::MatrixXd::Index minIndex;
        double minVal = eigenvals.col(0).minCoeff(&minIndex);
        return eigenvecs.col(minIndex);
    }


    Eigen::VectorXd ClusterLogger::computeSmallestEigenvector(sensor_msgs::PointCloud2 &cluster)
    {
        // get cluster variance
        Eigen::MatrixXd cov = computeClusterVariance(cluster);
        Eigen::VectorXd eigenvals = cov.eigenvalues().real();
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::MatrixXd eigenvecs = es.eigenvectors().real();
        Eigen::MatrixXf::Index minIndex;
        double minVal = eigenvals.col(0).minCoeff(&minIndex);
        return eigenvecs.col(minIndex);
    }

} // namespace logging
