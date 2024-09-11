/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

#include <stdio.h>
#include "utilities.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"

#define TEST_GPIOX GPIOA


#define TEST_PIN_1   GPIO_PIN_14
#define DHT11_PIN GPIO_PIN_4 // Define the GPIO pin connected to DHT11

#define BUZZER_PIN GPIO_PIN_9

#define ACTIVE_REGION LORAMAC_REGION_AS923
#define ST_LED_PIN GPIO_PIN_15
/*!
 * Defines the application data transmission duty cycle. 4s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            4000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0


#define LORAWAN_DBM                                 TX_POWER_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/*!
 * LoRaWAN application port
 */
//
#define LORAWAN_APP_PORT                            2
   
static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif
int a = 0;
/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 4;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           16

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime = APP_TX_DUTYCYCLE;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
 bool NextTx = true;

/*!
 * Device states
 */

int counter = 0; 
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

void dht11_init(void) {
    gpio_init(GPIOA, DHT11_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
    gpio_write(GPIOA, DHT11_PIN, GPIO_LEVEL_HIGH);
}

void dht11_start_signal(void) {
    gpio_write(GPIOA, DHT11_PIN, GPIO_LEVEL_LOW);
    delay_ms(18); // Pull the pin low for at least 18ms
    gpio_write(GPIOA, DHT11_PIN, GPIO_LEVEL_HIGH);
    delay_us(20); // Then pull it high for 20-40us
    gpio_init(GPIOA, DHT11_PIN, GPIO_MODE_INPUT_FLOATING); // Set pin as input
}

bool dht11_check_response(void) {
    delay_us(40); // Wait for 40us
    if (gpio_read(GPIOA, DHT11_PIN) == GPIO_LEVEL_LOW) {
        delay_us(80); // Wait for 80us
        if (gpio_read(GPIOA, DHT11_PIN) == GPIO_LEVEL_HIGH) {
            delay_us(50); // Wait for 50us
            return true;
        }
    }
    return false;
}

uint8_t dht11_read_byte(void) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        while (gpio_read(GPIOA, DHT11_PIN) == GPIO_LEVEL_LOW); // Wait for the pin to go high
        delay_us(30); // Wait for 30us
        if (gpio_read(GPIOA, DHT11_PIN) == GPIO_LEVEL_HIGH) {
            result |= (1 << (7 - i)); // Read the bit (high)
            while (gpio_read(GPIOA, DHT11_PIN) == GPIO_LEVEL_HIGH); // Wait for the pin to go low
        }
    }
    return result;
}

void dht11_read_data(uint8_t* temperature, uint8_t* humidity) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    dht11_start_signal();
    if (dht11_check_response()) {
        for (int i = 0; i < 5; i++) {
            data[i] = dht11_read_byte();
        }
        if (data[4] == (data[0] + data[1] + data[2] + data[3])) { // Checksum
            *humidity = data[0];
            *temperature = data[2];
        }
    }
}

/*!
 * \brief   Prepares the payload of the frame
 */
void PrepareTxFrame(uint8_t port, int counter) {

    uint8_t raw_temperature = 0, raw_humidity = 0;
    dht11_read_data(&raw_temperature, &raw_humidity);

    // Scale temperature and humidity by 100
    uint16_t temperature = raw_temperature * 100;
    uint16_t humidity = raw_humidity * 100;
    if (counter >= 10){
        AppDataSize = 5;
        AppData[0] = 0x00; // Payload type identifier
        AppData[1] = (temperature >> 8) & 0xFF; // High byte of temperature
        AppData[2] = temperature & 0xFF;        // Low byte of temperature
        AppData[3] = (humidity >> 8) & 0xFF;    // High byte of humidity
        AppData[4] = humidity & 0xFF;
    }
    else
    {
        AppDataSize = 2;
        AppData[0] = 0x01;
        if (gpio_read_output(GPIOA,TEST_PIN_1)== 1)
        {
            AppData[1] = 0x01;
        }
        else{
            AppData[1] = 0x00;
        }
               // Low byte of humidity
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Đặt NextTx thành true để cho phép gửi dữ liệu tiếp theo
                NextTx = true;
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                // Đặt NextTx thành true để cho phép gửi dữ liệu tiếp theo
                NextTx = true;
                break;
            }
            case MCPS_PROPRIETARY:
            {
                // Xử lý yêu cầu Proprietary nếu cần
                NextTx = true;
                break;
            }
            default:
                // Xử lý yêu cầu không xác định nếu cần
                NextTx = false;
                break;
        }
    }
    else
    {
        // Xác nhận không thành công, đặt NextTx thành false để không gửi dữ liệu tiếp theo
        NextTx = false;
    }
}


/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication(McpsIndication_t *mcpsIndication)
{
    static uint8_t threshold_temperature = 80; // Bien luu nguong gia tri nhiet do
    uint8_t raw_temperature = 0, raw_humidity = 0;
    dht11_read_data(&raw_temperature, &raw_humidity);
    a = 1;
    if(a==1)
    {
        gpio_write(GPIOA, ST_LED_PIN, GPIO_LEVEL_HIGH);
    }
    printf("McpsIndication function called\n");

    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
        printf("McpsIndication: Status not OK. Status = %d\n", mcpsIndication->Status);
        return;
    }

    printf("Receive data: rssi = %d, snr = %d, datarate = %d, port = %d\r\n", 
           mcpsIndication->Rssi, (int)mcpsIndication->Snr, 
           (int)mcpsIndication->RxDatarate, mcpsIndication->Port);

    // Check type data
    const char *mcpsIndicationStr;
    switch (mcpsIndication->McpsIndication)
    {
        case MCPS_UNCONFIRMED:
            mcpsIndicationStr = "MCPS_UNCONFIRMED";
            break;
        case MCPS_CONFIRMED:
            mcpsIndicationStr = "MCPS_CONFIRMED";
            break;
        case MCPS_PROPRIETARY:
            mcpsIndicationStr = "MCPS_PROPRIETARY";
            break;
        case MCPS_MULTICAST:
            mcpsIndicationStr = "MCPS_MULTICAST";
            break;
        default:
            mcpsIndicationStr = "UNKNOWN";
            break;
    }
    printf("Data type: %s\n", mcpsIndicationStr);

    // Check FramePending
    if (mcpsIndication->FramePending)
    {
        printf("FramePending is true\n");
        OnTxNextPacketTimerEvent();
    }

    // Check received data
    if (mcpsIndication->RxData)
    {
        printf("Data received. Buffer size = %d\n", mcpsIndication->BufferSize);
        if (mcpsIndication->Buffer != NULL)
        {
            for (int i = 0; i < mcpsIndication->BufferSize; i++)
            {
                printf("Data[%d] = %02X\n", i, mcpsIndication->Buffer[i]);
            }

            // Check value BufferSize
            if (mcpsIndication->BufferSize >= 2)
            {
                switch (mcpsIndication->Buffer[0])
                {
                    case 0x0B:
                        switch (mcpsIndication->Buffer[1])
                        {
                            case 0x05:
                                printf("Change status LED\n");
                                gpio_toggle(TEST_GPIOX, TEST_PIN_1);
                                break;                    
                            default:
                                printf("Unknown command in buffer\n");
                                break;
                        }

                    default:
                        printf("Unknown command in buffer\n");
                        break;
                }
            }
            if (mcpsIndication->BufferSize == 1)
            {
                threshold_temperature = mcpsIndication->Buffer[0];
                printf("New threshold temperature set: %d\n", threshold_temperature);
            }
        }
        else
        {
            printf("No data in buffer\n");
        }
    }
    else
    {
        printf("No data received\n");
    }

    // Compare the current temperature with the threshold temperature and activate buzzer if necessary
    if (raw_temperature > threshold_temperature)
    {
        printf("Temperature %d exceeds threshold %d, activating buzzer\n", raw_temperature, threshold_temperature);
        gpio_write(GPIOA, BUZZER_PIN, GPIO_LEVEL_LOW); // Assuming BUZZER_GPIO_PORT and BUZZER_PIN are defined
    }
    else
    {
        gpio_write(GPIOA, BUZZER_PIN, GPIO_LEVEL_HIGH); // Turn off the buzzer if temperature is below threshold
    }
}


/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */

static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
    switch (mlmeConfirm->MlmeRequest)
    {
        case MLME_JOIN:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                printf("joined\r\n");
                DeviceState = DEVICE_STATE_SEND;
                NextTx = true; // Allow transmission
            }
            else
            {
                printf("join failed\r\n");
                // Retry joining the network
                DeviceState = DEVICE_STATE_JOIN;
                NextTx = false; // Do not allow transmission until rejoining is successful
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                // Link check successful
                // Check DemodMargin
                // Check NbGateways
                NextTx = true; // Allow transmission
            }
            else
            {
                printf("Link check failed\r\n");
                // Retry joining the network
                DeviceState = DEVICE_STATE_JOIN;
                NextTx = false; // Do not allow transmission until rejoining is successful
            }
            break;
        }
        default:
            NextTx = false; // Do not allow transmission for unknown requests
            break;
    }
}


/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        case MLME_JOIN:
        {
            if( mlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                printf("joined\r\n");
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                printf("join failed\r\n");
                DeviceState = DEVICE_STATE_JOIN; // Retry joining the network
            }
            break;
        }
        default:
            break;
    }
}

static void lwan_dev_params_update( void )
{
    MibRequestConfirm_t mibReq;
    uint16_t channelsMaskTemp[6];
    channelsMaskTemp[0] = 0x00FF;
    channelsMaskTemp[1] = 0x0000;
    channelsMaskTemp[2] = 0x0000;
    channelsMaskTemp[3] = 0x0000;
    channelsMaskTemp[4] = 0x0000;
    channelsMaskTemp[5] = 0x0000;

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}


/**
 * Main application entry point.
 */
int app_start(void)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    gpio_init(TEST_GPIOX, BUZZER_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
    gpio_init(TEST_GPIOX, ST_LED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(TEST_GPIOX, TEST_PIN_1, GPIO_MODE_OUTPUT_PP_LOW);
    DeviceState = DEVICE_STATE_INIT;

    printf("ClassA app start\r\n");
    // Initialization

    while (1)
    {
        dht11_init();
        switch (DeviceState)
        {
            case DEVICE_STATE_INIT:
            {
                printf("1\n");
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION);

                TimerInit(&TxNextPacketTimer, OnTxNextPacketTimerEvent);

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm(&mibReq);

                lwan_dev_params_update();

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                printf("2\n");
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                // BoardGetUniqueId(DevEui);

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 8;

                if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK)
                {
                    printf("2-1\n");
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    printf("2-2\n");
                    DeviceState = DEVICE_STATE_CYCLE;
                }
                break;
            }
            case DEVICE_STATE_SEND:
            {
                counter++;
                
                printf("3\n");
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort, counter );

                    NextTx = SendFrame();
                    if(SendFrame())
                    {
                        if(a == 0)
                        {
                            gpio_write(GPIOA, ST_LED_PIN, GPIO_LEVEL_LOW);
                        }
                        a = 0;
                    }                   
                }
                if (counter == 10)
                {
                    counter = 0;
                }
                
            }
            case DEVICE_STATE_CYCLE:
            {
                printf("4\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue(&TxNextPacketTimer, TxDutyCycleTime);
                TimerStart(&TxNextPacketTimer);
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler();

                // Process Radio IRQ
                Radio.IrqProcess();
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
