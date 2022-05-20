#include "RFID.h"
#include "gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variable
 ******************************************************************************/
__CHAR32_TYPE__ data_ready;

/*******************************************************************************
 * Functions
 ******************************************************************************/
static void RFID_MsgHandler(service_t *service, msg_t *msg);


void RFID_Init(void)
{
    revision_t revision = {.major = 1, .minor = 0, .build = 0};

    Luos_CreateService(RFID_MsgHandler, RFID_TYPE, "rfid", revision);
}

void RFID_Loop(void)
{
  data_ready = LinearOD_PositionFrom_mm(rfid_calcul());
}

static void RFID_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == GET_CMD)
    {
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target      = msg->header.source;
        LinearOD_PositionToMsg(&data_ready, &pub_msg);
        Luos_SendMsg(service, &pub_msg);
    }
}