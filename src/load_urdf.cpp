#include <iostream>
#include <dart/dart.h>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>

static const std::string topicName("dart_markers");

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "error: incorrect number of arguments\n"
                 "usage: load_urdf package://my_package/path/to/robot.urdf"
              << std::endl;
    return 1;
  }

  const std::string urdfUri(argv[1]);

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever
    = std::make_shared<aikido::util::CatkinResourceRetriever>();

  // Load the HERB URDF model as a Skeleton.
  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
    urdfUri, resourceRetriever);

  if (!skeleton)
  {
    std::cerr << "Failed loading HERB from URI '" << urdfUri << "'\n";
    return 1;
  }

  // Print some debug information.
  std::cout << "BodyNodes:\n";
  for (dart::dynamics::BodyNode *bodyNode : skeleton->getBodyNodes())
    std::cout << "- " << bodyNode->getName() << '\n';

  std::cout << std::endl;

  std::cout << "DegreeOfFreedoms:\n";
  for (dart::dynamics::DegreeOfFreedom *dof : skeleton->getDofs())
    std::cout << "- " << dof->getName() << '\n';

  std::cout << std::endl;

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_herb");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
            << "' InteractiveMarker topic in RViz." << std::endl;
  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
  return 0;
}
