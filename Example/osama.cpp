/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>

 pcl::visualization::PCLVisualizer visualize (pcl::visualization::PCLVisualizer viewer, 
                                                            std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > &newClouds,
                                                            std::string cloud_initial_name,
                                                            bool OPT_RED,
                                                            bool OPT_BIG_POINTS);
using namespace pcl;
using namespace std;

int
main (int argc, char *argv[])
{
/*  ///The smallest scale to use in the DoN filter.
  double scale1;
  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;
*/
  /*if (argc < 6)
  {
    cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << endl;
    exit (EXIT_FAILURE);
  }*/

  /// the file to read from.
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  //viewer.setFullScreen(true); 
  viewer.setCameraPosition(-21.1433, -23.4669, 12.7822,0.137915, -0.429331, -1.9301,0.316165, 0.28568, 0.904669);
  viewer.setCameraClipDistances(0.0792402, 79.2402); 
  int num_frames;
  int initial_frame;
  istringstream (argv[1]) >> initial_frame;
  istringstream (argv[2]) >> num_frames;
  //string infile = argv[1];
  for (int a=initial_frame;a<=num_frames;++a){
  /// small scale
  /*istringstream (argv[2]) >> scale1;
  /// large scale
  istringstream (argv[3]) >> scale2;
  istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  istringstream (argv[5]) >> segradius;   // threshold for radius segmentation
*/
  // Load cloud in blob format
  //pcl::PCLPointCloud2 blob;
  //pcl::io::loadPCDFile (infile.c_str (), blob);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_original (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  std::string filename2; 
  std::stringstream sa;
  sa << setw(6) << setfill('0') << a;
  filename2= sa.str();
  reader.read ("../Data/pcd-files/KITTI/" + filename2 + ".pcd", *cloud_original);
  //reader.read (infile.c_str (), *cloud_original);
  //pcl::fromPCLPointCloud2 (blob, *cloud);
  // eliminate far-away x points
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud_original);
  vg.setLeafSize (0.010f, 0.010f, 0.010f);
  vg.filter (*cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-30, 30);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // eliminate far-away y points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-30, 30);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // eliminate far-away z points
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.8, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.75);//0.75
  ec.setMinClusterSize (10);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  // cluster_temp contains the cluster that we input into clusters
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_temp;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
                                                 it != cluster_indices.end (); ++it)
  {
    cluster_temp.reset (new pcl::PointCloud<pcl::PointXYZ>());

    for (std::vector<int>::const_iterator pit = it->indices.begin ();
                                          pit != it->indices.end (); ++pit)
    {
      cluster_temp->points.push_back (cloud->points[*pit]); 
    }
    if ((cluster_temp->points.size () > 250)){
      continue;
    }

    cluster_temp->width = cluster_temp->points.size ();
    cluster_temp->height = 1;
    cluster_temp->is_dense = true;

    // Add cluster
    clusters.push_back(cluster_temp);
    // std::cout << "Cluster " << clusters.size() - 1 << " has " << clusters.back()->points.size () 
    //           << " points." << std::endl;

    // If we want to save to clusters to disk - false default
    if (p.SWITH_WRITE_CLUSTERS)
    {
      pcl::PCDWriter writer;
      std::stringstream ss;
      ss << "cloud_cluster_" << clusters.size() - 1 << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *clusters.back(), false); 
    }
  } */

  // cout<< "# Features extracted before eliminating the planes is: "
  //     << clusters.size()<< endl;

  /*if (cloud->points.size()>0){
  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  float radius = 5.51;

  kdtree.setInputCloud (cloud);
  pcl::PointXYZ searchPoint;
  searchPoint.x= 0;
  searchPoint.y= 0;
  searchPoint.z= 0;

  std::vector<int> idx;
  std::vector<float> pointRadiusSquaredDistance;
  if ( kdtree.radiusSearch (searchPoint, radius, idx, pointRadiusSquaredDistance) > 0 )
  {
    newCloud->points.resize( idx.size() );
    for (size_t i = 0; i < idx.size (); ++i)
    {
      newCloud->points[i]= cloud->points[ idx[i] ];
    }
  }

  *cloud= *newCloud;
  
  cout<< "Cloud filtered"<< endl;
}*/
  viewer.removeAllPointClouds();
  // Update cloud original - White cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>white_color (cloud_original, 255, 255, 255);                             
  viewer.addPointCloud <pcl::PointXYZ>(cloud_original, white_color,"cloud_original");

  // Create a search tree, use KDTreee for non-organized data.
  /*pcl::search::Search<PointXYZ>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZ> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);
  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);*/

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
/*  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);
*/
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  std::stringstream ss;
  int j=0;
  double area;
  double density;
  //int k=0;
  int g;
  double volume;
  double sumx, sumy, sumz, sumx2, sumy2, sumz2;
  double slinderness;
  double symmtry;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it,++j)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (cloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;
    clusters.push_back(cloud_cluster_don);
    ss << j;
    //area= pcl::calculatePolygonArea (*clusters[j]);
    sumx=0;
    sumy=0;
    sumz=0;
    for (g=0;g<(clusters[j]->points.size());++g){
      sumx=sumx+double(clusters[j]->points[g].x);
      sumy=sumy+double(clusters[j]->points[g].y);
      sumz=sumz+double(clusters[j]->points[g].z);
    }
    sumx=sumx/double(clusters[j]->points.size());
    sumy=sumy/double(clusters[j]->points.size());
    sumz=sumz/double(clusters[j]->points.size());
    sumx2=0;
    sumy2=0;
    sumz2=0;
    for (g=0;g<(clusters[j]->points.size());++g){
      sumx2=sumx2+(double(clusters[j]->points[g].x)-sumx)*(double(clusters[j]->points[g].x)-sumx);
      sumy2=sumy2+(double(clusters[j]->points[g].y)-sumy)*(double(clusters[j]->points[g].y)-sumy);
      sumz2=sumz2+(double(clusters[j]->points[g].z)-sumz)*(double(clusters[j]->points[g].z)-sumz);}
    sumx2=sumx2/double(clusters[j]->points.size());
    sumy2=sumy2/double(clusters[j]->points.size());
    sumz2=sumz2/double(clusters[j]->points.size());
    volume=sumx2*sumy2*sumz2;
    slinderness=sumz2/sqrt((sumx2*sumx2)+(sumy2*sumy2));
    symmtry=abs(sumx2-sumy2);
    density=double(clusters[j]->points.size())/(volume*1000);
    //cout << "sumz2= " << sumz2 << endl;
    //density= (clusters[j]->points.size()) / area;
    if (/*(clusters[j]->points.size()>=12)&&*//*(clusters[j]->points.size()<1100)&&*//*(symmtry<0.01)&&*/(density>500)&&(slinderness>3 /*try 2*/)&&(volume<0.0001)/*&&(area<3)*/&&(sumz2>0.1)){
    //k=k+1;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>red_color (clusters[j], 255, 0, 0); 
    viewer.addPointCloud <pcl::PointXYZ>(clusters[j], red_color,"clusters["+ ss.str()+"]");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "clusters["+ ss.str()+"]");
    cout << "j=" << j << endl;
    //Save cluster
    cout << "PointCloud representing the Cluster" << j << ": " << clusters[j]->points.size () << " data points." << std::endl;
    cout << "volume[" << j << "]=" << volume << endl;
    cout << "sumz2= " << sumz2 << endl;
    cout << "symmtry= " << symmtry << endl;
    cout << "slindeness= " << slinderness << endl;
    cout << "density= " << density << endl;
    }
    /*cout << "j=" << j << endl;
    //Save cluster
    cout << "PointCloud representing the Cluster" << j << ": " << clusters[j]->points.size () << " data points." << std::endl;*/
    //stringstream ss;
    //ss << "don_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_don, false);
  }
  //cout << "k=" << k << endl;
  viewer.spinOnce ();
  //while (!viewer.wasStopped ())
    //{
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
      //viewer.spinOnce ();
    //}
  }
  while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    viewer.spinOnce ();
    }
}