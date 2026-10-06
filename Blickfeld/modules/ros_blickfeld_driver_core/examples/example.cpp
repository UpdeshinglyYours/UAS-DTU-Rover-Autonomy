#include <string>

#include <boost/program_options.hpp>

#include "blickfeld_driver_core/blickfeld_driver_core.h"

class ExampleDriver : public blickfeld::ros_interop::BlickfeldDriverCore {
 public:
  using blickfeld::ros_interop::BlickfeldDriver::BlickfeldDriverCore;
  void publishPointCloud(__attribute__((unused)) PointCloud2Ptr pointcloud) override {
    std::cout << "Publishing PointCloud Message" << std::endl;
  }
  void publishStatus(__attribute__((unused)) DiagnosticStatusPtr scan_pattern) {
    std::cout << "Publishing Diagnostic Message!" << std::endl;
  }
  void printDebugInfo(const blickfeld::ros_interop::DebugMessages& debug_messages) {
    for (const auto& debug_message : debug_messages) {
      std::cout << debug_message.first << ": " << std::endl << debug_message.second.str() << std::endl;
    }
  }
};

int main(int argc, char** argv) {
  boost::program_options::options_description options("Allowed options");
  options.add_options()("help", "produce help message")("host", boost::program_options::value<std::string>(),
                                                        "set host name");
  boost::program_options::variables_map commandline_variables;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, options), commandline_variables);
  boost::program_options::notify(commandline_variables);
  auto commandline_options = std::make_pair(options, commandline_variables);

  if (commandline_options.second.count("help")) {
    std::cout << commandline_options.first << std::endl;
    return 1;
  }

  std::string host;
  if (!commandline_options.second.count("host")) {
    std::cout << "Please specify a host device with the --host option." << std::endl;
    std::cout << commandline_options.first << std::endl;
    return 1;
  } else {
    host = commandline_options.second["host"].as<std::string>();
  }

  ExampleDriver cube(host);
  cube.start();

  while (1)
    ;

  return 0;
}
