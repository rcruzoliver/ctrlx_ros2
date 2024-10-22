// Author: Raul Cruz-Oliver
// Email: raul.cruz.oliver@gmail.com
// Date, place: January 2023, Butikkon, CH

// Node implementations
#include "nodeCollection.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << "[jog_demo]" << std::endl;
		return 1;
	}

	std::string option = argv[1];

	if (option == "jog_demo") {
		std::cout << "Jog demo state manager has started." << std::endl;
	} else {
		std::cerr << "Invalid option. Use 'jog_demo' argument to start the demostration example." << std::endl;
	}

	if (option == "jog_demo") {
		rclcpp::spin(std::make_shared<ScaraStateManager>());
	} else {
		return 1;
	}

	rclcpp::shutdown();
	return 0;
}
