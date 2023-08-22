

extern esp_spp_cb_param_t spp_param;
extern char *ComBuff, *BTBuff;
extern char ComByte, BTByte;
extern uint8_t BTstateCount ;
extern bool updateBT,updateSerial,partnerState;
extern uint16_t partnerTime,sendOnlineTime;

void sendBluetooth(char *string);
void delay(uint16_t t);
void setLoraWork(void);
void sendUart(char *string,uint16_t len);
void sendBluetooth(char *string);
void setLoraConf(void);
void Send2Phone(char *string, uint8_t Lora);
void WaitFreeLora(void);
void WaitSendLora(void);
void setParamLora(uint8_t airDrate);
void parsConfig(char *string);
int indexOf(char *source,char *target);
uint8_t LastindexOf(char *source,int target);
void offLED(void);
void TaskPhoneConnected(bool PhoneReady);
void TaskDataPhoneReceive(char *string,uint16_t len);
void TaskDataLoraReceive(char *string, uint16_t len);
void checkPartner(void);