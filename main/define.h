/***************Bluetooth*************/
#define BTname "HTA_LORA_2"

#define BTconnected 20
#define BTdisconnected 3

#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA /*Choose show mode: show data or speed*/

/**************GPIO-Pin*********************/
// #define M0pin 32
// #define M1pin 26
// #define LED_btpin 22
// #define LED_RXpin 15
// #define LED_TXpin 13
// #define LED_Partnerpin 14
// #define AUXpin 12
// #define RXpin 16
// #define TXpin 17

#define M0pin 21
#define M1pin 19
#define LED_btpin 2
#define LED_RXpin 14
#define LED_TXpin 13
#define LED_Partnerpin 12
#define AUXpin 18
#define RXpin 16
#define TXpin 17

/**************GPIO-Set*********************/
#define M0(x) gpio_set_level(M0pin, x);
#define M1(x) gpio_set_level(M1pin, x);
#define LED_bt(x) gpio_set_level(LED_btpin, x);
#define LED_RX(x) gpio_set_level(LED_RXpin, x);
#define LED_TX(x) gpio_set_level(LED_TXpin, x);
#define LED_Partner(x) gpio_set_level(LED_Partnerpin, x);
#define LoraBusy !gpio_get_level(AUXpin)
#define LoraSend gpio_get_level(AUXpin)

/***************String****************/
#define ConfigDelivery "\"@#configok\r\n"
#define ConfigStartString "@#config>"
#define PartnerString "@#online"
#define DeliveryString  "\"@#delivery"
#define PhoneDelivery "\"@#ok\r\n"
#define LoraFreeError "\"Lora can't free\r\n"
#define LoraSendError "\"Lora can't Send\r\n"


/***************UART****************/
#define EX_UART_NUM         UART_NUM_2
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define LoraBaudRate        9600
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

/***************Timer****************/
#define TimerMicroSec       400 * 1000      // period = 400 mSec
#define TimeWaitFreeLora    9967295
#define PartnerTimeCheck         5 * 2.5         //  5 Sec
#define SendOnlineTime         3 * 2.5         //  3 Sec

