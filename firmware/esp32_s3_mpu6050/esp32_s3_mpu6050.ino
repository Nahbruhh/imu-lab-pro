#include <Wire.h>
#define MPU 0x68
#define SDA_PIN 5
#define SCL_PIN 4
uint16_t hz=200;
uint8_t rangeG=2, dlpf=3;
uint16_t gyroRange=250;
uint32_t nextTick=0, seq=0, readErrors=0, overruns=0;
String cmd="";
bool wr(uint8_t r,uint8_t v){Wire.beginTransmission(MPU);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
bool rd(uint8_t r,uint8_t &v){Wire.beginTransmission(MPU);Wire.write(r);if(Wire.endTransmission(false)!=0)return false;if(Wire.requestFrom(MPU,1,true)!=1)return false;v=Wire.read();return true;}
bool configure(){
 uint8_t a=(rangeG==2?0:rangeG==4?8:rangeG==8?16:24);
 uint8_t g=(gyroRange==250?0:gyroRange==500?8:gyroRange==1000?16:24);
 uint8_t div=1000/hz-1;
 bool ok=wr(0x6B,1)&&wr(0x6C,0)&&wr(0x1A,dlpf)&&wr(0x19,div)&&wr(0x1C,a)&&wr(0x1B,g);
 uint8_t ca=0,cg=0,cd=0,cs=0;
 ok=ok&&rd(0x1C,ca)&&rd(0x1B,cg)&&rd(0x1A,cd)&&rd(0x19,cs);
 ok=ok&&((ca&24)==a)&&((cg&24)==g)&&((cd&7)==dlpf)&&(cs==div);
 if(ok){nextTick=micros()+1000000UL/hz;seq=0;readErrors=0;overruns=0;}
 Serial.printf("# CONFIG %u %u %u %u %s\n",rangeG,hz,dlpf,gyroRange,ok?"OK":"ERROR");
 return ok;
}
void command(String s){s.trim();int a,b,c,d;int n=sscanf(s.c_str(),"CFG %d %d %d %d",&a,&b,&c,&d);
 if(n==3)d=250;
 if(n>=3 && (a==2||a==4||a==8||a==16) && (b==50||b==100||b==200||b==500||b==1000) && c>=0&&c<=6 && (d==250||d==500||d==1000||d==2000)){
  uint8_t oa=rangeG,oc=dlpf;uint16_t ob=hz,od=gyroRange;
  rangeG=a;hz=b;dlpf=c;gyroRange=d;
  if(!configure()){rangeG=oa;hz=ob;dlpf=oc;gyroRange=od;configure();}
 }else Serial.println("# BAD_CONFIG");
}
void setup(){Serial.begin(460800);delay(1200);Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(400000);
 uint8_t id=0;if(!rd(0x75,id)||id!=0x68){Serial.printf("# MPU_ID_ERROR 0x%02X\n",id);while(true)delay(1000);}
 Serial.printf("# WHO_AM_I 0x%02X\n",id);if(!configure())while(true)delay(1000);
 Serial.println("# STREAM_START");
}
void loop(){while(Serial.available()){char c=Serial.read();if(c=='\n'||c=='\r'){if(cmd.length()){command(cmd);cmd="";}}else if(cmd.length()<60)cmd+=c;}
 uint32_t now=micros();if((int32_t)(now-nextTick)<0)return;
 uint32_t interval=1000000UL/hz;nextTick+=interval;
 if((int32_t)(now-nextTick)>=0){overruns++;nextTick=now+interval;}
 Wire.beginTransmission(MPU);Wire.write(0x3B);
 if(Wire.endTransmission(false)!=0||Wire.requestFrom(MPU,14,true)!=14){readErrors++;return;}
 int16_t v[7];for(int i=0;i<7;i++)v[i]=(int16_t)((Wire.read()<<8)|Wire.read());
 Serial.printf("%lu,%lu,%d,%d,%d,%d,%d,%d\n",(unsigned long)seq++,(unsigned long)micros(),v[0],v[1],v[2],v[4],v[5],v[6]);
}
