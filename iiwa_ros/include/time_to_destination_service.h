#pragma once

#include <iiwa_msgs/TimeToDestination.h>
#include <iiwa_services.hpp>

namespace iiwa_ros {
	
	class TimeToDestinationService : public iiwaServices<iiwa_msgs::TimeToDestination> {
	public:
		TimeToDestinationService(const std::string& service_name, const bool verbose = true);
		double getTimeToDestination();
	protected:
		virtual bool callService();
	private:
		double time_to_destination_;
	};
	
}