#ifndef ECMASTER_H_
#define ECMASTER_H_

#include <rtt/PropertyBag.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>
#include <ethercatprint.h>

#include "ethercat_driver.h"

class ECMaster : public RTT::TaskContext {
public:
	ECMaster(const std::string& name);
	~ECMaster();

	bool configureHook();
	void cleanupHook();
	bool startHook();
	void updateHook();
	void stopHook();	
private:
  void printIOMAP();
  void ecError(int ret);

  ecx_contextt ec_context_;
  char io_map_[4096];
  
  ec_slavet ec_slave_[EC_MAXSLAVE];
  /** number of slaves found on the network */
  int ec_slavecount_;
  /** slave group structure */
  ec_groupt ec_group_[EC_MAXGROUP];

  /** cache for EEPROM read functions */
  uint8 esibuf_[EC_MAXEEPBUF];
  /** bitmap for filled cache buffer bytes */
  uint32 esimap_[EC_MAXEEPBITMAP];
  /** current slave for EEPROM cache buffer */
  ec_eringt ec_elist_;
  ec_idxstackT ec_idxstack_;

  /** SyncManager Communication Type struct to store data of one slave */
  ec_SMcommtypet ec_SMcommtype_;
  /** PDO assign struct to store data of one slave */
  ec_PDOassignt ec_PDOassign_;
  /** PDO description struct to store data of one slave */
  ec_PDOdesct ec_PDOdesc_;

  /** buffer for EEPROM SM data */
  ec_eepromSMt ec_SM_;
  /** buffer for EEPROM FMMU data */
  ec_eepromFMMUt ec_FMMU_;

  int64 ec_DCtime_;
  boolean EcatError_;
  ecx_portt ecx_port_;
  ecx_redportt ecx_redport_;
  
  std::vector<ECDriver *> drivers_;
  
  std::string iface_;
  
  RTT::Property<RTT::PropertyBag> slaves_property_;
  std::vector<std::string> names_;
};

#endif  // ECMASTER_H_

