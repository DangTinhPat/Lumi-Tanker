#ifndef HCR_PROTOCOL_H
#define HCR_PROTOCOL_H
/* Portable C99/C++ codec for ESP32-S3. Wire floats are IEEE754 float32 LE. */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#define HCR_VERSION 1
#define HCR_SCHEMA 0x31524348u
#define HCR_MAX_PAYLOAD 256
#define HCR_MAX_FRAME 280
#define HCR_TARGET_TTL_US 20000u
enum { HCR_HELLO=1,HCR_HELLO_ACK=2,HCR_TELEMETRY=3,HCR_TARGET=129,HCR_STATUS=130 };
enum { HCR_IMU_OK=1,HCR_ENCODER_OK=2,HCR_CALIBRATED=4,HCR_ESTOP=8,
       HCR_TORQUE_VALID=16,HCR_TARGET_VALID=32,HCR_DRIVER_FAULT=64 };
enum { HCR_DISABLED=0,HCR_HOME_PD=1,HCR_RECOVERY=2,HCR_POLICY_PD=3 };
enum { HCR_LEGS_ENABLE=1,HCR_WHEELS_ENABLE=2 };
enum { HCR_GYRO=0,HCR_GRAVITY=3,HCR_ACCEL=6,HCR_Q=9,HCR_QD=15,HCR_HIP_TORQUE=21,HCR_APPLIED_TARGET=23 };

typedef struct { uint8_t type; uint32_t seq; uint64_t stamp_us; uint16_t size; uint8_t payload[256]; } HcrFrame;
typedef struct { uint8_t bytes[280]; size_t used; uint32_t errors; } HcrParser;
typedef struct { uint32_t epoch,source_seq; uint16_t profile,flags; float target[6],feedforward[4]; } HcrTarget;

static inline uint16_t hcr_u16(const uint8_t*p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static inline uint32_t hcr_u32(const uint8_t*p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static inline uint64_t hcr_u64(const uint8_t*p){return hcr_u32(p)|((uint64_t)hcr_u32(p+4)<<32);}
static inline void hcr_put16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static inline void hcr_put32(uint8_t*p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static inline void hcr_put64(uint8_t*p,uint64_t v){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(v>>(8*i));}
static inline float hcr_float(const uint8_t*p){uint32_t u=hcr_u32(p);float f;memcpy(&f,&u,4);return f;}
static inline void hcr_putfloat(uint8_t*p,float f){uint32_t u;memcpy(&u,&f,4);hcr_put32(p,u);}
static inline int hcr_newer(uint32_t a,uint32_t b){uint32_t d=a-b;return d>0&&d<0x80000000u;}
static inline uint32_t hcr_crc32(const uint8_t*p,size_t n){
  uint32_t c=0xffffffffu;
  for(size_t i=0;i<n;i++){c^=p[i];for(int k=0;k<8;k++)c=(c>>1)^(0xedb88320u&((uint32_t)0-(c&1)));}
  return c^0xffffffffu;
}
static inline size_t hcr_encode(uint8_t*out,size_t cap,uint8_t type,uint32_t seq,uint64_t stamp,const uint8_t*payload,uint16_t size){
  if(size>256||cap<(size_t)size+24)return 0;
  out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=type;
  hcr_put16(out+4,size);hcr_put16(out+6,0);hcr_put32(out+8,seq);hcr_put64(out+12,stamp);
  if(size)memcpy(out+20,payload,size);
  hcr_put32(out+20+size,hcr_crc32(out,20+size));return 24+size;
}
/* Feed one byte. Returns 1 when a complete frame is available. Bounded memory. */
static inline int hcr_feed(HcrParser*p,uint8_t byte,HcrFrame*out){
  if(p->used==280){memmove(p->bytes,p->bytes+1,--p->used);p->errors++;}
  p->bytes[p->used++]=byte;
  while(p->used>=2){
    if(p->bytes[0]!=0xa5||p->bytes[1]!=0x5a){memmove(p->bytes,p->bytes+1,--p->used);continue;}
    if(p->used<20)return 0;
    uint16_t size=hcr_u16(p->bytes+4);
    if(p->bytes[2]!=1||hcr_u16(p->bytes+6)||size>256){p->errors++;memmove(p->bytes,p->bytes+1,--p->used);continue;}
    size_t total=24+size;if(p->used<total)return 0;
    if(hcr_u32(p->bytes+20+size)!=hcr_crc32(p->bytes,20+size)){p->errors++;memmove(p->bytes,p->bytes+1,--p->used);continue;}
    out->type=p->bytes[3];out->seq=hcr_u32(p->bytes+8);out->stamp_us=hcr_u64(p->bytes+12);out->size=size;
    memcpy(out->payload,p->bytes+20,size);p->used-=total;memmove(p->bytes,p->bytes+total,p->used);return 1;
  }
  return 0;
}
static inline size_t hcr_hello_ack(uint8_t*out,size_t cap,uint32_t seq,uint64_t stamp,uint32_t epoch){
  uint8_t p[12];hcr_put32(p,epoch);hcr_put16(p+4,200);hcr_put16(p+6,100);hcr_put32(p+8,HCR_SCHEMA);
  return hcr_encode(out,cap,HCR_HELLO_ACK,seq,stamp,p,12);
}
static inline size_t hcr_telemetry(uint8_t*out,size_t cap,uint32_t seq,uint64_t stamp,uint32_t epoch,uint32_t flags,uint32_t applied_seq,const float values[29]){
  uint8_t p[128];hcr_put32(p,epoch);hcr_put32(p+4,flags);hcr_put32(p+8,applied_seq);
  for(unsigned i=0;i<29;i++)hcr_putfloat(p+12+4*i,values[i]);
  return hcr_encode(out,cap,HCR_TELEMETRY,seq,stamp,p,128);
}
static inline int hcr_target(const HcrFrame*f,HcrTarget*out){
  if(f->type!=HCR_TARGET||f->size!=52)return 0;
  out->epoch=hcr_u32(f->payload);out->source_seq=hcr_u32(f->payload+4);
  out->profile=hcr_u16(f->payload+8);out->flags=hcr_u16(f->payload+10);
  if(out->profile>3||(out->flags&~3))return 0;
  for(unsigned i=0;i<6;i++){out->target[i]=hcr_float(f->payload+12+4*i);if(!isfinite(out->target[i]))return 0;}
  for(unsigned i=0;i<4;i++){out->feedforward[i]=hcr_float(f->payload+36+4*i);if(!isfinite(out->feedforward[i]))return 0;}
  return 1;
}
#endif
