#ifndef ECDRIVER_H_
#define ECDRIVER_H_

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>
#include <ethercatprint.h>

#include <rtt/Service.hpp>
#include <rtt/TaskContext.hpp>

class ECDriver {
public:
  ECDriver(const std::string &name, ecx_contextt* contextt_ptr, uint16_t slave_id) :
      contextt_ptr_(contextt_ptr),
      slave_id_(slave_id),
      name_(name),
      device_name_(contextt_ptr->slavelist[slave_id].name),
      service_(new RTT::Service(name_)) {
    device_name_ = std::string(contextt_ptr->slavelist[slave_id].name);
    service_->addConstant("device_name", device_name_);
    service_->addConstant("state", contextt_ptr->slavelist[slave_id].state);
  }
  
  virtual ~ECDriver() {
    service_->clear();
  }
  
  virtual bool configure() {
    return true;
  }
  
  virtual bool start() {
    return true;
  }
  
  virtual void update() {
  }
  
  virtual void stop() {
  }
  
  RTT::Service::shared_ptr provides() {
    return service_;
  }
  
protected:
  ecx_contextt* contextt_ptr_;
  uint16_t slave_id_;

  std::string name_;
  std::string device_name_;
  RTT::Service::shared_ptr service_;
};

#endif // ECDRIVER_H_
