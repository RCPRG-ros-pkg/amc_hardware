#if !defined(AMC_SUBNODE_H)
#define AMC_SUBNODE_H

#include <rtt/Port.hpp>

#include <ethercattype.h>

class AMCSubNode {
public:
	AMCSubNode(const std::string &neme, ecx_contextt* contextt_ptr, uint16_t slave_id, int subnode_id);
	~AMCSubNode();
	bool configure();
	bool start();
	void update();
	void stop();
	
	size_t getRPDOSize();
	size_t getTPDOSize();
	
	void setRPDOPtr(uint8_t * ptr);
	void setTPDOPtr(uint8_t * ptr);
	
	RTT::Service::shared_ptr provides() {
    return service_;
  }
	
	std::string name_;
	
	enum DriveState {Unknown, NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, Fault, QuickStopActive};
	enum StateCommand {ResetFault, DisableVoltage, ShutDown, SwitchOn, EnableOperation, QuickStop, BeginHoming};
	enum Mode {ProfilePosition=0x01, ProfileVelocity=0x03, ProfileTorque=0x04, Homing=0x06, CyclicSynchronousPosition=0x08, CyclicSynchronousVelocity=0x09, CyclicSynchronousTorque=0x0A, Config0=0x9E, Config1=0xDE, None=0xFF};
	
private:

  bool beginHoming();
  bool enableOperation();
  
	bool isHomingCompleted();
	DriveState translateState(uint16 statusWord);
	bool enterStateNoCheck(StateCommand cmd);
  void processStatus();
	DriveState getState();
	bool enterState(StateCommand cmd);
  
  
  bool readConversionConstants();

  int32_t DA1toDrive(double da1);
  double DrivetoDA1(int32 drive);
  int32_t DS4toDrive(double ds4);
  double DrivetoDS4(int32 drive);
  int16_t DC2toDrive(double dc2);
  double DrivetoDC2(int16 drive);
  
  // EC structures
  int subnode_id_;
  uint16_t slave_id_;
  ecx_contextt* contextt_ptr_;
  
  RTT::Service::shared_ptr service_;
  
  // conversion factors
  double conv_DA1_;
	double conv_DS4_;
	double conv_DC2_;
	
	// PDO
	
	struct __attribute__ ((__packed__)) RPDO_pos {
	  uint16_t control_word;
	  int32_t target_position;
	  int16_t current_offset;
	  int32_t velocity_offset;
	  int16_t user_bits;
	  int16_t mode_of_operation;
	};
	
	struct __attribute__ ((__packed__)) RPDO_vel {
	  uint16_t control_word;
	  int32_t target_velocity;
	  int16_t current_offset;
	  int16_t user_bits;
	  int16_t mode_of_operation;
	};
	
	struct __attribute__ ((__packed__)) RPDO_cur {
	  uint16_t control_word;
	  int16_t target_current;
	  int16_t user_bits;
	  int16_t mode_of_operation;
	};
	
	union __attribute__ ((__packed__)) RPDO_t {
	  RPDO_pos position;
	  RPDO_vel velocity;
	  RPDO_cur current;
	};
	
	struct __attribute__ ((__packed__)) TPDO_t {
	  int16_t status_word;
	  int32_t actual_position;
	  int32_t actual_velocity;
	  int16_t actual_current;
	  uint16_t digital_inputs;
	};
	
	RPDO_t * rpdo_ptr_;
  TPDO_t * tpdo_ptr_;
  
  Mode getInitMode();
	void setControlMode(Mode mode);
	Mode getMode();
	
  void setCurrent(double current);
	void setVelocity(double velocity);
	void setPosition(double position);
	double getActualCurrent();
	double getActualVelocity();
	double getActualPosition();
	
	void setDO(unsigned int n);
	void resetDO(unsigned int n);
	
	// Internal
	
	Mode mode_;
	Mode actualMode_;
		
  bool homing_compleat_;
	int servo_state_;
	
	bool enable_;
	bool homing_;
	
	std::string mode_param_;
	
	StateCommand target_state_;
	
	bool position_valid_;
	
	// RTT Ports
  RTT::OutputPort<double> port_motor_position_;
	RTT::OutputPort<double> port_motor_velocity_;
	RTT::OutputPort<double> port_motor_current_;
	
	RTT::InputPort<double> port_motor_position_command_;
	RTT::InputPort<double> port_motor_velocity_command_;
	RTT::InputPort<double> port_motor_current_command_;
	
	RTT::InputPort<bool> port_do1_command_;
	RTT::InputPort<bool> port_do2_command_;
	RTT::InputPort<bool> port_do3_command_;
	RTT::InputPort<bool> port_do4_command_;
	
};

#endif  //AMC_SUBNODE_H
