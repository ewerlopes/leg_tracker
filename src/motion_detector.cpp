#include <player_tracker/motion_detector.h>



MotionDetector::MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename)
        : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration), on_window(false), window(window_duration*10) {
    ros::NodeHandle nh_private("~");

    base_frame_ = "odom";

    initRosComunication();

    log_file.open(log_filename);
}

    /**
    * @basic Constructor
    * @param nh A nodehandle
    * @param scan_topic the scan topic we would like to use
    * @param window_duration the slide windows duration in seconds
    */
MotionDetector::MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration)
    : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration), on_window(false),
    window(window_duration*10){
    ros::NodeHandle nh_private("~");

    base_frame_ = "odom";

    initRosComunication();
}

void MotionDetector::initRosComunication()
{
    scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
    leg_cluster_sub = nh_.subscribe("detected_leg_clusters", 1, &MotionDetector::legClusterCallback, this);

    merged_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("merged_cloud", 10);
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
    merged_cloud_cluster_pub = nh_.advertise<sensor_msgs::PointCloud2>("merged_clusters", 10);
    leg_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("leg_cloud", 10);
    markers_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

    ros::service::waitForService("assemble_scans");
    client = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
}

ros::NodeHandle& MotionDetector::nodeHandle()
{
    return this->nh_;
}

MotionDetector::~MotionDetector(){
    if (log_file.is_open()){
        log_file.close();
    }
}

sensor_msgs::PointCloud MotionDetector::getPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud(base_frame_, *scan_msg, cloud, tf_listener);
    return cloud;
}

void MotionDetector::publishMergedCloud(ros::Time from, ros::Time to){
    sensor_msgs::PointCloud cloud;

    if (mergeWindowClouds(from, to, cloud)){
        merged_cloud_pub.publish(cloud);
    }else{
        ROS_ERROR("Failed to publish merged cloud!");
    }
}

void MotionDetector::publishMergedCloud(sensor_msgs::PointCloud cloud){
    merged_cloud_pub.publish(cloud);
}

void MotionDetector::publishEigenMarker(std_msgs::Header header, Eigen::VectorXd eigenvect, int marker_id, geometry_msgs::Point32 mean){
    // Publish marker to rviz
    visualization_msgs::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = header.frame_id;
    m.ns = "Eigen";
    m.id = marker_id;
    m.type = m.ARROW;
    geometry_msgs::Point new_mean;
    new_mean.x = mean.x;
    new_mean.y = mean.y;
    new_mean.z = mean.z;
    m.points.push_back(new_mean);

    if (eigenvect(2) > 0)
        eigenvect = eigenvect * -1;

    geometry_msgs::Point eigen_point;
    eigen_point.x = eigenvect(0) + new_mean.x;
    eigen_point.y = eigenvect(1) + new_mean.y;
    eigen_point.z = eigenvect(2) + new_mean.z;
    m.points.push_back(eigen_point);
    //m.pose.position.x = eigenvect(0);
    //m.pose.position.y = eigenvect(1);
    //m.pose.position.z = eigenvect(2);
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;
    m.lifetime = ros::Duration(0.1);

    #pragma omp critical
    markers_pub.publish(m);
}

int MotionDetector::mergeWindowClouds(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud){
    srv.request.begin = from;
    srv.request.end   = to;

    ROS_WARN("Time diff: %f", (srv.request.end.toSec() - start_time.toSec()));

    if (client.call(srv)){
        ROS_INFO("Got cloud with %lu points\n", srv.response.cloud.points.size());
        cloud = srv.response.cloud;
        return 1;
    }else{
        ROS_ERROR("Service call to assemble_scans failed\n");
        return 0;
    }
}

void MotionDetector::setTemporal(sensor_msgs::PointCloud &cloud, double time){
    #pragma omp parallel for
    for (int i=0; i < cloud.points.size(); i++){
        cloud.points[i].z = time;
    }
}

void MotionDetector::saveToLog(std::string text){
        if (log_file.is_open()){
        log_file << text.c_str() << std::endl;
    }
}

void MotionDetector::saveCloudDataToLog(sensor_msgs::PointCloud &cloud){
    if (log_file.is_open()){
        for (int i=0; i < cloud.points.size(); i++){
            log_file << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
        }
    }
}

void MotionDetector::legClusterCallback(const player_tracker::LegArray::ConstPtr &leg_clusters_msg){
    #pragma omp parallel for
    for (int i=0; i < leg_clusters_msg->legs.size(); i++){
        sensor_msgs::PointCloud cloud;
        cloud.header = leg_clusters_msg->header;
        cloud.points = leg_clusters_msg->legs[i].points;

        ROS_WARN("leg cloud %d size %d", i, leg_clusters_msg->legs[i].points.size());

        try{
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0.0), ros::Duration(0.1));
            tf_listener.transformPointCloud("odom", cloud, cloud);
            setTemporal(cloud, (ros::Time::now().toSec() - start_time.toSec()));
            #pragma omp critical
            leg_cloud_pub.publish(cloud);
        }catch (std::exception ex){
            ROS_ERROR("%s",ex.what());
        }
    }
}

geometry_msgs::Point32 MotionDetector::getClusterMean(sensor_msgs::PointCloud2 &input){
    sensor_msgs::PointCloud cloud;
    if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
        geometry_msgs::Point32 sum;
        float factor = (float) 1/cloud.points.size();
        for (int i=0; i < cloud.points.size(); i++){
            sum.x += cloud.points[i].x * factor;
            sum.y += cloud.points[i].y * factor;
            sum.z += cloud.points[i].z * factor;
        }
        return sum;
    }else{
        ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
    }
}

Eigen::MatrixXd MotionDetector::getProjection(Eigen::MatrixXd &dataset, Eigen::VectorXd &eigenvect){
    return eigenvect.transpose() * dataset;
}

Eigen::MatrixXd MotionDetector::getCloudPointsAsMatrix(sensor_msgs::PointCloud2 &input) {
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

Eigen::MatrixXd MotionDetector::getCloudPointsAsMatrix(sensor_msgs::PointCloud &cloud) {
    Eigen::MatrixXd output;
    #pragma omp parallel for
    for (int i=0; i < cloud.points.size(); i++){
        output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
    }
    return output;
}


Eigen::MatrixXd MotionDetector::getClusterVariance(Eigen::MatrixXd &input) {
    Eigen::MatrixXd centered = input.colwise() - input.rowwise().mean();
    Eigen::MatrixXd cov = (centered * centered.transpose()) / double(input.cols() - 1);
    return cov;
}


Eigen::MatrixXd MotionDetector::getClusterVariance(sensor_msgs::PointCloud2 &input){
    sensor_msgs::PointCloud cloud;
    if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
        Eigen::MatrixXd point_matrix = getCloudPointsAsMatrix(input);
        return getClusterVariance(point_matrix);
    }else{
        ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
    }

}

Cloud2List MotionDetector::getClusters(sensor_msgs::PointCloud2 &input){

    Cloud2List clusters;

    pcl::PCLPointCloud2 to_convert;

    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl_conversions::toPCL(input, to_convert);

    pcl::PointCloud<pcl::PointXYZ>::Ptr converted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(to_convert,*converted);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(converted);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.50); // 50cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(converted);
    ec.extract(cluster_indices);

    int count(0);

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

    ROS_WARN("Number of clusters: %d", count);
    return clusters;
}

Eigen::VectorXd MotionDetector::getClusterSmallestEigenValue(Eigen::MatrixXd cov){
    Eigen::VectorXd eigenvals = cov.eigenvalues().real();
    Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
    Eigen::MatrixXd eigenvecs = es.eigenvectors().real();

    Eigen::MatrixXd::Index minIndex;
    double minVal = eigenvals.col(0).minCoeff(&minIndex);

    ROS_INFO_STREAM("EIGENVALUES:\n " << eigenvals);
    ROS_INFO_STREAM("minVal: " << minVal << "\tminIndex: " << minIndex);
    return eigenvecs.col(minIndex);
}


Eigen::VectorXd MotionDetector::getClusterSmallestEigenValue(sensor_msgs::PointCloud2 &cluster){
    // get cluster variance
    Eigen::MatrixXd cov = getClusterVariance(cluster);
    Eigen::VectorXd eigenvals = cov.eigenvalues().real();
    Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
    Eigen::MatrixXd eigenvecs = es.eigenvectors().real();


    Eigen::MatrixXf::Index minIndex;
    double minVal = eigenvals.col(0).minCoeff(&minIndex);


    ROS_INFO_STREAM("EIGENVALUES:\n " << eigenvals);
    ROS_INFO_STREAM("minVal: " << minVal << "\tminIndex: " << minIndex);
    return eigenvecs.col(minIndex);
}

Eigen::MatrixXd::Index MotionDetector::getSteepestVector(Eigen::MatrixXd eigenvectors){
    Eigen::MatrixXd A = eigenvectors.topRows(2);
    Eigen::VectorXd B = A.colwise().squaredNorm();
    Eigen::MatrixXd::Index minIndex;
    B.minCoeff(&minIndex);
    return minIndex;
}


void MotionDetector::saveProjectionToFile(Eigen::MatrixXd projected, int id){
    if (log_file.is_open()){
        #pragma omp critical
        for (int i=0; i < projected.cols(); i++){
            log_file << "(" << id << "," << projected(i) << ");";
        }
    }

}


void MotionDetector::processWindows(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

    Cloud2List clusters = extractClusterInWindow();
    
    if (clusters.empty()) {
        ROS_WARN("ON processWindows: No clusters found. Skipping...");
        return;
    }

    Eigen::VectorXd direction = computeSteepestDirection(clusters);

    // Save to file
    for(int i=0; i < clusters.size(); i++){
        Eigen::MatrixXd asMatrix = getCloudPointsAsMatrix(clusters[i]);       //Contains to_merge_cloud as matrix.
        Eigen::MatrixXd projected = getProjection(asMatrix, direction);
        // ROS_INFO_STREAM(asMatrix.cols() << " <?> " << projected.cols());
        if (projected.cols() != clusters[i].width * clusters[i].height){
            ROS_ERROR("NOT EQUAL");
        }
        saveProjectionToFile(projected, i);
    }
}

Cloud2List MotionDetector::extractClusterInWindow() {
    sensor_msgs::PointCloud to_merge_cloud;
    sensor_msgs::PointCloud2 as_pointcloud_2;

    int mergedClouds = mergeWindowClouds(start_time, ros::Time::now(), to_merge_cloud);


    // if (to_merge_cloud.points.empty()){
    //     ROS_FATAL("Not merging");
    //     ros::requestShutdown();

    // }

    if (!mergedClouds) {
        return Cloud2List();
    }

    int convertedClouds = sensor_msgs::convertPointCloudToPointCloud2(to_merge_cloud, as_pointcloud_2);
    if (!convertedClouds) {
        return Cloud2List();
    }

    Cloud2List clusters = getClusters(as_pointcloud_2);
    return clusters;
}

Eigen::VectorXd MotionDetector::computeSteepestDirection(Cloud2List &clusters) {
    Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;
    #pragma omp parallel for
    for(int i=0; i < clusters.size(); i++){
        ROS_INFO("Processing cluster %d", i);
        Eigen::VectorXd eigenvect = getClusterSmallestEigenValue(clusters[i]);
        eigenvects.col(i) << eigenvect(0), eigenvect(1), eigenvect(2);

        #pragma omp critical
        merged_cloud_cluster_pub.publish(clusters[i]);
        geometry_msgs::Point32 mean = getClusterMean(clusters[i]);
        publishEigenMarker(clusters[i].header, eigenvect, i, mean);
    }


    Eigen::MatrixXd::Index steepest_eigenvect = getSteepestVector(eigenvects);
    return eigenvects.col(steepest_eigenvect);
}

void MotionDetector::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

    // check whether we can transform data
    try{
        if (!tf_listener.waitForTransform(scan_msg->header.frame_id,
            base_frame_, scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() *
            scan_msg->time_increment), ros::Duration(1.0))) {
            return;
        }
    }catch (std::exception ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    sensor_msgs::PointCloud cloud;

    if (!on_window) {

        ROS_WARN("Starting windows...");
        on_window = true;
        start_time = scan_msg->header.stamp;

    } else if ((scan_msg->header.stamp.toSec() - start_time.toSec()) < window_duration_.toSec()) {

        ROS_INFO("Inside windows...");
        ROS_INFO("Time: %f", (scan_msg->header.stamp.toSec() - start_time.toSec()));

    } else {

        ROS_WARN("End of windows... Processing");
        processWindows(scan_msg);
        saveToLog("\n--");
        on_window = false;
        clearWindowCloudList();
        return;

    }

    cloud = getPointCloud(scan_msg);
    appendWindowCloudList(cloud);

    // set time dimension
    setTemporal(cloud, (scan_msg->header.stamp.toSec() - start_time.toSec()));

    // publish point cloud
    cloud_pub.publish(cloud);
}

PointCloudList& MotionDetector::getWindowCloudList()
{
    return cloudsInWindow;
}

void MotionDetector::clearWindowCloudList()
{
    cloudsInWindow.clear();
}

void MotionDetector::appendWindowCloudList(sensor_msgs::PointCloud &cloud)
{
    cloudsInWindow.push_back(cloud);
}

void MotionDetector::run()
{ /*    */ }
