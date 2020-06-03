#include "mbed.h"
#include "mbed_rpc.h"
#include "math.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
WiFiInterface *wifi;
InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
// RawSerial pc(USBTX, USBRX);
Serial pc(USBTX, USBRX);
RawSerial xbee(D12, D11);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue2(32 * EVENTS_EVENT_SIZE);
InterruptIn sw2(SW2);
Thread t;
Thread t2;


#define UINT14_MAX        16383
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1)
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1)
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1)
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1)
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7
I2C i2c( PTD9,PTD8);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);

void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void rpc_call(Arguments *in, Reply *out);
void acce_value();
void messageArrived(MQTT::MessageData& md);
void close_mqtt();
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
RPCFunction rpc_acce(&rpc_call, "rpc_call");

Timer timer1;
float tt[3], origin[3];
int hi = 0, hi2 = 0;
float hv[500];

int main(){

  pc.baud(9600);
  char xbee_reply[4];
  xbee.baud(9600);
  xbee.printf("+++");
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
  }
  xbee.printf("ATMY 0x240\r\n");
  reply_messange(xbee_reply, "setting MY : <REMOTE_MY>");
  xbee.printf("ATDL 0x241\r\n");
  reply_messange(xbee_reply, "setting DL : <REMOTE_DL>");
  xbee.printf("ATID 0x0\r\n");
  reply_messange(xbee_reply, "setting PAN ID : <PAN_ID>");
  xbee.printf("ATWR\r\n");
  reply_messange(xbee_reply, "write config");
  xbee.printf("ATCN\r\n");
  reply_messange(xbee_reply, "exit AT mode");
  xbee.getc();
  pc.printf("start\r\n");
  
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  t2.start(callback(&queue2, &EventQueue::dispatch_forever));
  queue2.call_every(100,&acce_value);

  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
}
void xbee_rx_interrupt(void){
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}
void xbee_rx(void){
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    memset(buf, 0, 100);
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    
    RPC::call(buf, outbuf);
    // wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}
void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void acce_value(){
      uint8_t who_am_i, data[2], res[6];
      int16_t acc16;
      FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
      data[1] |= 0x01;
      data[0] = FXOS8700Q_CTRL_REG1;
      FXOS8700CQ_writeRegs(data, 2);
      FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
      acc16 = (res[0] << 6) | (res[1] >> 2);
      if (acc16 > UINT14_MAX/2)
          acc16 -= UINT14_MAX;
      tt[0] = ((float)acc16) / 4096.0f;
      acc16 = (res[2] << 6) | (res[3] >> 2);
      if (acc16 > UINT14_MAX/2)
          acc16 -= UINT14_MAX;
      tt[1] = ((float)acc16) / 4096.0f;
      acc16 = (res[4] << 6) | (res[5] >> 2);
      if (acc16 > UINT14_MAX/2)
          acc16 -= UINT14_MAX;
      tt[2] = ((float)acc16) / 4096.0f;

      hv[hi] = sqrt(pow(abs(tt[0]*10),2) + pow(abs(tt[1]*10),2));
      hi ++;
}

void rpc_call(Arguments *in, Reply *out){
  int hi3 = hi;
  while(hi2<=hi3){
    xbee.printf("%.2f\n", hv[hi2]);
     hi2 ++;
  }
  hi2--;
}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}
void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

void messageArrived(MQTT::MessageData& md) {
      MQTT::Message &message = md.message;
      char msg[300];
      sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
      printf(msg);
      wait_ms(1000);
      char payload[300];
      sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
      printf(payload);
      ++arrivedcount;
}

void close_mqtt() {
      closed = true;
}