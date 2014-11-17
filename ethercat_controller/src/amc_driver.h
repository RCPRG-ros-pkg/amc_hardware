#if !defined(AMC_DRIVER_H)
#define AMC_DRIVER_H

#include "ethercat_driver.h"

#include <rtt/Port.hpp>

#include <ethercattype.h>

#include "amc_subnode.h"

class AMCDriver : public ECDriver {
public:
	AMCDriver(const std::string &name, ecx_contextt* contextt_ptr, uint16_t slave_id);
	~AMCDriver();
	bool configure();
	bool start();
	void update();
	void stop();
	void cleanup();
private:
  void printTPDO();
  int getNumberOfSubnodes();
  int subnodes_;
  std::vector<AMCSubNode *> subnode_;
};

#endif  //AMC_DRIVER_H
