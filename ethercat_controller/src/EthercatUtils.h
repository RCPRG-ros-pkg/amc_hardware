bool ethercatInit(const char *ifname);
void ethercatClose();
int getSlaveCount();
bool ethercatEnterOperationalState();
bool ethercatEnterOperationalStateNoBlock();
bool ethercatEnterSafeOpState();
void ethercatSend();
void ethercatReceive();
void ethercatSync();

