#include <iostream>
#include <dart/dart.h>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>

static const std::string topicName("dart_markers");

int main(int argc, char **argv)
{

	const std::string robotUrdfUri(argv[1]);
	const std::string objectUrdfFolder(argv[2]);
	const std::string configFilePath(argv[3]);

	// Resolves package:// URIs by emulating the behavior of 'catkin_find'.
	const auto resourceRetriever
	= std::make_shared<aikido::util::CatkinResourceRetriever>();

	// Load the HERB URDF model as a Skeleton.
	dart::utils::DartLoader urdfLoader;
	const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
	robotUrdfUri, resourceRetriever);

	if (!skeleton)
	{
		std::cerr << "Failed loading HERB from URI '" << robotUrdfUri << "'\n";
		return 1;
	}

	std::cout << "BodyNodes:\n";
  	for (dart::dynamics::BodyNode *bodyNode : skeleton->getBodyNodes())
    	std::cout << "- " << bodyNode->getName() << '\n';

    dart::dynamics::BodyNode *herb_base_node = skeleton->getBodyNode("/herb_base");

  	std::cout << std::endl;

  	std::cout << "Starting ROS node now!" << std::endl;
  	ros::init(argc, argv, "test_aikido_perception");
  	ros::NodeHandle nh("~");

  	aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  	viewer.addSkeleton(skeleton);
  	viewer.setAutoUpdate(true);

  	//Test perception

  	aikido::perception::AprilTagsModule at_detector(nh,"/apriltags_kinect2/marker_array",
  				configFilePath, resourceRetriever,	objectUrdfFolder,"herb_base",
  				herb_base_node);

  	std::vector< dart::dynamics::SkeletonPtr > skeleton_list;

  	at_detector.detectObjects(skeleton_list,10.0);
  	std::cout << "Detection don - " << skeleton_list.size()<<" skeletons!"<<std::endl;
  	
  	for (dart::dynamics::SkeletonPtr skel : skeleton_list){
  		viewer.addSkeleton(skel);
  	}

  	ros::spin();
  	return 0;
}