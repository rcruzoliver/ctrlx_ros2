// Author: Raul Cruz-Oliver
// Date, place: Novemeber 2023, Butikkon, CH

// Author: Raul Cruz-Oliver
// Email: raul.cruz.oliver@gmail.com
// Date, place: January 2023, Butikkon, CH

// Node implementations
#include "nodeCollection.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ScaraStatePublisher>());
	rclcpp::shutdown();
	return 0;
}
