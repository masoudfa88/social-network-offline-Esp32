#include "includes.h"

void delay(uint16_t t)
{
    vTaskDelay(t / portTICK_PERIOD_MS);
}

void setLoraWork()
{
    M0(0);
    M1(0);
    delay(500);
}

void setLoraConf()
{
    M0(0);
    M1(1);
    delay(500);
}

void sendUart(char *string, uint16_t len)
{
    uart_write_bytes(EX_UART_NUM, (const char *)string, len);
    sendOnlineTime = 0;
}

void sendBluetooth(char *string)
{
    esp_spp_write(spp_param.data_ind.handle, strlen(string), (const char *)string);
}

void Send2Phone(char *string, uint8_t Lora)
{
    if (BTstateCount == BTconnected)
    {
        sendBluetooth(string);
        ESP_LOGE("send2phone", "%s", string);
    }
    else
        ESP_LOGI("Bluetooth", "Phone not connected");
}

void WaitFreeLora(void)
{
    unsigned int TimeCounter = 0;
    while (LoraBusy)
    {
        TimeCounter++;
        if (TimeCounter > TimeWaitFreeLora)
        {
            Send2Phone(LoraFreeError, 2);
            ESP_LOGE("LORA", LoraFreeError);
            break;
        }
    }
}

void WaitSendLora(void)
{
    unsigned int TimeCounter = 0;
    while (LoraSend)
    {
        TimeCounter++;
        if (TimeCounter > TimeWaitFreeLora)
        {
            Send2Phone(LoraSendError, 2);
            ESP_LOGE("LORA", LoraSendError);
            break;
        }
    }
}

void setParamLora(uint8_t airDrate)
{
    uint8_t config[] = {0xc0, 0x00, 0x09, 0x00, 0x00, 0x00, 0x60, 0x00, 0x17, 0x03, 0x00, 0x00};
    config[6] += airDrate;
    delay(1000);
    WaitFreeLora();
    setLoraConf();
    delay(500);
    WaitFreeLora();
    uart_write_bytes(EX_UART_NUM, (const char *)config, 12);
    WaitSendLora();
    delay(500);
    WaitFreeLora();
    setLoraWork();
}

char *substr(const char *src, int m, int n)
{
    // get the length of the destination string
    int len = n - m;

    // allocate (len + 1) chars for destination (+1 for extra null character)
    char *dest = (char *)malloc(sizeof(char) * (len + 1));

    // extracts characters between m'th and n'th index from source string
    // and copy them into the destination string
    for (int i = m; i < n && (*(src + i) != '\0'); i++)
    {
        *dest = *(src + i);
        dest++;
    }

    // null-terminate the destination string
    *dest = '\0';

    // return the destination string
    return dest - len;
}

int indexOf(char *source, char *target)
{
    /* assume source address is */
    /* 0x10 for example */
    char *found = strstr(source, target); /* should return 0x18 */
    if (found != NULL)                    /* strstr returns NULL if item not found */
    {
        uint8_t index = found - source; /* index is 8 */
        return index;
    }
    return -1;
}

uint8_t LastindexOf(char *source, int target)
{
    /* assume source address is */
    /* 0x10 for example */
    char *found = strrchr(source, target); /* should return 0x18 */
    if (found != NULL)                     /* strstr returns NULL if item not found */
    {
        uint8_t index = found - source; /* index is 8 */
        return index;
    }
    return 0;
}

void parsConfig(char *string)
{
    uint8_t airDrate = atoi(substr(string, indexOf(string, "A") + 1, LastindexOf(string, 'A')));
    setParamLora(airDrate);
    Send2Phone(ConfigDelivery, 0);
    ESP_LOGE("LORA", "Config Sent");
}

void offLED(void)
{
    LED_bt(0)
        LED_TX(0)
            LED_RX(0)
}

void TaskPhoneConnected(bool PhoneReady)
{
    if (PhoneReady)
        BTstateCount = BTconnected;
    else
        BTstateCount = BTdisconnected;
}

void TaskDataPhoneReceive(char *string, uint16_t len)
{
    updateBT = true;
    WaitFreeLora();
    sendUart(string, len);
    WaitSendLora();
    if (indexOf(string, ConfigStartString) >= 0)
        parsConfig(string);
    else if (!(indexOf(string, DeliveryString) >= 0))
    {
        Send2Phone(PhoneDelivery, 0);
        ESP_LOGE("send2Phone", "normal data");
    }
}

void TaskDataLoraReceive(char *string, uint16_t len)
{
    partnerTime = 0;
    partnerState = true;
    checkPartner();
    if (indexOf(string, ConfigStartString) >= 0)
    {
        parsConfig(string);
        updateSerial = true;
    }
    else if (indexOf(string, PartnerString) >= 0)
    {
        partnerState = true;
        partnerTime = 0;
    }
    else
    {
        Send2Phone(string, 1);
        updateSerial = true;
    }
    ESP_LOGE("send2Phone", "TaskDataLoraReceive");
}

void checkPartner(void)
{
    LED_Partner(partnerState);
    partnerTime = 0;
}