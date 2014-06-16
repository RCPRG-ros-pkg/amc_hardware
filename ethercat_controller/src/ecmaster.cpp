
#include "ecmaster.h"
#include "AxisDrive.h"
#include <rtt/Component.hpp>

ECMaster::ECMaster(const std::string& name) : RTT::TaskContext(name), EcatError_(FALSE), iface_("rteth0"), slaves_property_("slaves", "slaves") {

 ec_context_.port = &ecx_port_;
 ec_context_.slavelist = &ec_slave_[0];
 ec_context_.slavecount = &ec_slavecount_;
 ec_context_.maxslave = EC_MAXSLAVE;
 ec_context_.grouplist = &ec_group_[0];
 ec_context_.maxgroup = EC_MAXGROUP;
 ec_context_.esibuf = &esibuf_[0];
 ec_context_.esimap = &esimap_[0];
 ec_context_.esislave = 0;
 ec_context_.elist = &ec_elist_;
 ec_context_.idxstack = &ec_idxstack_;
 ec_context_.ecaterror = &EcatError_;
 ec_context_.DCtO = 0;
 ec_context_.DCl = 0;
 ec_context_.DCtime = &ec_DCtime_;
 ec_context_.SMcommtype = &ec_SMcommtype_;
 ec_context_.PDOassign = &ec_PDOassign_;
 ec_context_.PDOdesc = &ec_PDOdesc_;
 ec_context_.eepSM = &ec_SM_;
 ec_context_.eepFMMU = &ec_FMMU_;
 
 this->provides()->addConstant("state", ec_slave_[0].state);
 this->provides()->addProperty("iface", iface_);
 this->properties()->addProperty("names", names_);
}

ECMaster::~ECMaster() {
}

bool ECMaster::configureHook() {
  int ret;
  ret = ecx_init(&ec_context_, iface_.c_str());
  if(ret < 0) {
    ecError(ret);
    return false;
  }
  
  ret = ecx_config_init(&ec_context_, FALSE);
  if(ret < 0) {
    ecError(ret);
    return false;
  }
   
  ret = ecx_config_map_group(&ec_context_, io_map_, 0);
  if(ret < 0) {
    ecError(ret);
    return false;
  }
  
  RTT::log(RTT::Info) << ec_slavecount_ << " slaves found and configured."
                    << RTT::endlog();
  
  RTT::log(RTT::Info) << "Request pre-operational state for all slaves"
                    << RTT::endlog();
  
  ec_slave_[0].state = EC_STATE_PRE_OP;
  ret = ecx_writestate(&ec_context_, 0);
  ecError(ret);
  ret = ecx_statecheck(&ec_context_, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
  ecError(ret);
  
  if(names_.size() != ec_slavecount_) {
    RTT::log(RTT::Error) << "size of names " << names_.size() << " does not match number of devices on the bus " << ec_slavecount_ << RTT::endlog();
    return false;
  }
  
  drivers_.resize(ec_slavecount_);
  
  for (int i = 1; i <= ec_slavecount_; i++) {
    drivers_[i-1] = new AxisDrive(names_[i-1], &ec_context_, i);
    this->provides()->addService(drivers_[i-1]->provides());
    if (!drivers_[i-1]->configure()) {
      return false;
    }
  }
  return true;
}

void ECMaster::cleanupHook() {
  for (unsigned int i = 0; i < drivers_.size(); i++){
    this->provides()->removeService(drivers_[i]->provides()->getName());
    delete drivers_[i];
  }
  
  ecx_close(&ec_context_);
}

bool ECMaster::startHook() {
  int ret = 0;

  RTT::log(RTT::Info) << "Request safe-operational state for all slaves" << RTT::endlog();
  ec_slave_[0].state = EC_STATE_SAFE_OP;
  ret = ecx_writestate(&ec_context_, 0);
  ecError(ret);
  
  // wait for all slaves to reach SAFE_OP state
  ret = ecx_statecheck(&ec_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  ecError(ret);
  
  if (ec_slave_[0].state != EC_STATE_SAFE_OP) {
    RTT::log(RTT::Error) << "Not all slaves reached safe operational state."
                        << RTT::endlog();
    ret = ecx_readstate(&ec_context_);
    ecError(ret);
    //If not all slaves operational find out which one
    for (int i = 0; i <= ec_slavecount_; i++) {
      if (ec_slave[i].state != EC_STATE_SAFE_OP) {
        RTT::log(RTT::Error) << "Slave " << i << " State= " <<
                                (int)ec_slave[i].state<< " StatusCode="
                                << ec_slave_[i].ALstatuscode << " : "
                                << ec_ALstatuscode2string(
                                        ec_slave_[i].ALstatuscode) << RTT::endlog();
      }
    }
    return false;
  }
  RTT::log(RTT::Info) << "Safe operational state reached for all slaves."
                      << RTT::endlog();
                      
  RTT::log(RTT::Info) << "Request operational state for all slaves" << RTT::endlog();
  ec_slave_[0].state = EC_STATE_OPERATIONAL;

  // send one valid process data to make outputs in slaves happy
  ret = ecx_send_processdata(&ec_context_);
  ecError(ret);
  
  ret = ecx_writestate(&ec_context_, 0);
  ecError(ret);

  // wait for all slaves to reach OP state
  ret = ecx_statecheck(&ec_context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
  ecError(ret);
  
  //if (ec_slave_[0].state != EC_STATE_OPERATIONAL) {
  //  RTT::log(RTT::Error) << "Not all slaves reached operational state."
  //             << RTT::endlog();
    ret = ecx_readstate(&ec_context_);
    //If not all slaves operational find out which one
    for (int i = 1; i <= ec_slavecount_; i++) {
      if (ec_slave_[i].state != EC_STATE_OPERATIONAL) {
        RTT::log(RTT::Error) << "Slave " << i << " State= " << 
                                (int)ec_slave_[i].state << " StatusCode="
                                << ec_slave_[i].ALstatuscode << " : "
                                << ec_ALstatuscode2string(
                                        ec_slave[i].ALstatuscode) << RTT::endlog();
      }
    }
  //  return false;
  //}
  RTT::log(RTT::Info) << "Operational state reached for all slaves."
                      << RTT::endlog();
                      
  for (int i = 0; i < ec_slavecount_; i++) {
    if(!drivers_[i]->start()) {
      return false;
    }
  }
                      
  return true;
}

void ECMaster::updateHook() {
    bool success = true;
    int ret = 0;
    if (ecx_receive_processdata(&ec_context_, EC_TIMEOUTRET) == 0) {
        success = false;
        RTT::log(RTT::Warning) << "receiving data failed" << RTT::endlog();
    }

    if (success)
        for (unsigned int i = 0; i < drivers_.size(); i++)
            drivers_[i]->update();

    if (ecx_send_processdata(&ec_context_) == 0) {
        success = false;
        RTT::log(RTT::Warning) << "sending process data failed" << RTT::endlog();
    }
}

void ECMaster::stopHook() {
}

void ECMaster::ecError(int ret) {
  if (ret < 0) {
    while (EcatError) {
      RTT::log(RTT::Error) << ec_elist2string() << RTT::endlog();
    }
  }
}
ORO_CREATE_COMPONENT(ECMaster)

