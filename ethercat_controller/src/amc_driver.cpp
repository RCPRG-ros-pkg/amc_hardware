
#include "amc_driver.h"

AMCDriver::AMCDriver(const std::string &name, ecx_contextt* contextt_ptr, uint16_t slave_id) :
  ECDriver(name, contextt_ptr, slave_id),
  subnodes_(0) {
  this->provides()->addOperation("printTPDO",	&AMCDriver::printTPDO, this, RTT::OwnThread);
}

AMCDriver::~AMCDriver() {
}

void AMCDriver::printTPDO() {
  for (int i = 0; i < contextt_ptr_->slavelist[slave_id_].Ibytes; i++) {
    printf("TPDO[%d] %X\n", i, (int)contextt_ptr_->slavelist[slave_id_].inputs[i]);
  }
}

bool AMCDriver::configure() {
  bool res = true;
  
  subnodes_ = getNumberOfSubnodes();

  int rpdo_size = 0;
  int tpdo_size = 0;

  uint8_t * rpdo = contextt_ptr_->slavelist[slave_id_].outputs;
  uint8_t * tpdo = contextt_ptr_->slavelist[slave_id_].inputs;
  
  RTT::log(RTT::Info) << "Creating " << subnodes_ << " subnodes" << RTT::endlog();
  
  subnode_.resize(subnodes_);
  for (int i = 0; i < subnodes_; i++) {
    subnode_[i] = new AMCSubNode(std::string("subnode") + (char)(i + (int)'0'), contextt_ptr_, slave_id_, i);
    service_->addService(subnode_[i]->provides());
    res = res && subnode_[i]->configure();
    
    subnode_[i]->setRPDOPtr(rpdo);
    subnode_[i]->setTPDOPtr(tpdo);
    
    rpdo += subnode_[i]->getRPDOSize();
    tpdo += subnode_[i]->getTPDOSize();
    
    rpdo_size += subnode_[i]->getRPDOSize();
    tpdo_size += subnode_[i]->getTPDOSize();
    
    if (rpdo_size > contextt_ptr_->slavelist[slave_id_].Obytes) {
      RTT::log(RTT::Error) << "RPDO error" << RTT::endlog();
      res = false;
      break;
    }
    
    if (tpdo_size > contextt_ptr_->slavelist[slave_id_].Ibytes) {
      RTT::log(RTT::Error) << "TPDO error" << RTT::endlog();
      res = false;
      break;
    }
  }
  
  memset(contextt_ptr_->slavelist[slave_id_].outputs, 0, rpdo_size);
  memset(contextt_ptr_->slavelist[slave_id_].inputs, 0, tpdo_size);
  
  return res;
}

bool AMCDriver::start() {
  bool ret = true;
  for (int i = 0; i < subnodes_; i++) {
    ret = ret && subnode_[i]->start();
  }
  
  return ret;
}

int AMCDriver::getNumberOfSubnodes() {
  uint8_t subnodes;
  int size = sizeof(subnodes);
  ecx_SDOread(contextt_ptr_, slave_id_, 0x1C12, 0x00, FALSE, &size, &subnodes, EC_TIMEOUTRXM);
  return (int)subnodes;
}

void AMCDriver::update() {
  for (int i = 0; i < subnodes_; i++) {
    RTT::Logger::In in(name_ + "." + subnode_[i]->name_);
    subnode_[i]->update();
  }
}

void AMCDriver::stop() {
  for (int i = 0; i < subnodes_; i++) {
    subnode_[i]->stop();
  }
}

void AMCDriver::cleanup() {
  for (int i = 0; i < subnodes_; i++) {
    delete subnode_[i];
  }
}

