#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>

#include <iostream>
#include <math.h>

using namespace std;

class AmcTest : public RTT::TaskContext{
private:

	RTT::OutputPort<std::vector<double> > port_cmd_current_;
	RTT::InputPort<std::vector<double> > port_current_;
	RTT::OutputPort<std::vector<double> > port_cmd_velocity_;
	RTT::InputPort<std::vector<double> > port_velocity_;
	RTT::OutputPort<std::vector<double> > port_cmd_position_;
	RTT::InputPort<std::vector<double> > port_position_;
	std::vector<double> cmd_current_;
	std::vector<double> current_;
	std::vector<double> cmd_velocity_;
	std::vector<double> velocity_;
	std::vector<double> cmd_position_;
	std::vector<double> position_;

	RTT::OperationCaller<bool()> beginOperation_method_;		// bool beginOperation()
	RTT::OperationCaller<bool(int)> startHoming_method_;		// bool startHoming(int)
	RTT::OperationCaller<bool(int)> isHomingComplete_method_;	// bool isHomingComplete(int)

	int counter_;
	double time_;

public:
	AmcTest(const std::string& name):
		TaskContext(name),
		counter_(0),
		time_(0)
	{
		cout<<"AmcTest()"<<endl;
		this->addPort("joint_current_command",	port_cmd_current_).doc("...");
		this->addPort("joint_current", 		port_current_).doc("...");
		this->addPort("joint_velocity_command",	port_cmd_velocity_).doc("...");
		this->addPort("joint_velocity",		port_velocity_).doc("...");
		this->addPort("joint_position_command",	port_cmd_position_).doc("...");
		this->addPort("joint_position",		port_position_).doc("...");


		cmd_current_.resize(1);
		cmd_current_[0] = 0.0;

		cmd_velocity_.resize(1);
		cmd_velocity_[0] = 0;

		cmd_position_.resize(1);
		cmd_position_[0] = 0;
	}

	~AmcTest()
	{
	}

	// RTT configure hook
	bool configureHook()
	{
		cout<<"AmcTest::configureHook"<<endl;

		TaskContext* a_task_ptr = getPeer("AmcDriver");
		if (a_task_ptr != NULL)
		{
			beginOperation_method_ = a_task_ptr->getOperation("beginOperation");		// bool beginOperation()
			startHoming_method_ = a_task_ptr->getOperation("startHoming");			// bool startHoming(int)
			isHomingComplete_method_ = a_task_ptr->getOperation("isHomingComplete");	// bool isHomingComplete(int)
		}
		cout << "a_task_ptr=" << a_task_ptr << endl; 
		return true;
	}

	// RTT start hook
	bool startHook()
	{
		cout<<"startHook"<<endl;
	}

	void stopHook()
	{
		cout<<"stopHook"<<endl;
	}

	// RTT update hook
	// This function runs every 1 ms (1000 Hz).
	void updateHook()
	{
		port_cmd_current_.write(cmd_current_);
		port_cmd_velocity_.write(cmd_velocity_);
		port_cmd_position_.write(cmd_position_);
		port_current_.read(current_);
		port_velocity_.read(velocity_);
		port_position_.read(position_);

		if (counter_<10)
		{
		}
		else if (counter_==10)
		{
			cout << "calling startHoming_method_(1) " << startHoming_method_.call(1) <<endl;
		}
		else if (counter_<20)
		{
			if (!isHomingComplete_method_.call(1))
			{
//				cout << "false" << endl;
				counter_ = 15;
			}
			else
			{
//				cout << "true" << endl;
			}
		}
		else if (counter_==20)
		{
			bool result = beginOperation_method_.call();
			cout << "calling beginOperation_method_() " << result <<endl;
		}
		else
		{
			double amplitude = 3.0;
			double period = 0.7;
			cmd_position_[0] = sin(2.0*M_PI*time_/period)*amplitude;
			time_ += 0.001;
		}

		if (current_.size() > 0 && velocity_.size() > 0 && position_.size() > 0)
		{
//			cout << "current: " << current_[0] << "\t\tvelocity: " << velocity_[0] << "\t\tposition: "<< position_[0] << endl;
//			cout << "current: " << current_[0] << endl;
		}
		else
		{
//			cout << "current_.size() < 1" << endl;
		}

		++counter_;
	}

};
ORO_CREATE_COMPONENT(AmcTest)


