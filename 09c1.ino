// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _NUM_SAMPLE 30//원하는 샘플의 개수(N)을 설정
// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, median; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int count = 0; // loop()를 도는 횟수
int N = _NUM_SAMPLE;
int middle;
float bigger,smaller;
float mid[_NUM_SAMPLE];
float org[_NUM_SAMPLE];
void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  median = 0.0;
  middle = (N-1)/2;  //홀수인경우 중위수위치,짝수인경우 2개의 중앙에 위치한수중 앞쪽에 위치한 수의 위치

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  if (count<N-1){
    org[count]=dist_raw;
    median = dist_raw;//초기2개의 값은 측정한 거리값
  }
  else if (count==N-1){
    org[count] = dist_raw;
    memcpy(mid,org,sizeof(float)*N);
    for (int i=0; i < N-1; i++){
       for (int k=0; k < (N-1)-i; k++){
           if (mid[k] > mid[k+1]){
               bigger = big(mid[k],mid[k+1]);
               smaller = small(mid[k],mid[k+1]);
               mid[k] = smaller;
               mid[k+1] = bigger;
               }
          }
      }
      if (N%2!=1){//N이 짝수인경우  
        median = (mid[middle] + mid[middle+1])/2;
      }
      else{//N이 홀수인 경우
        median = mid[middle];
      }
  }
  else {
    for (int i=0;i<N-1;i++){
      org[i] = org[i+1];
      }
      org[N-1] = dist_raw;

    memcpy(mid,org,sizeof(float)*N);
    for (int i=0; i < N-1; i++){
       for (int k=0; k < (N-1)-i; k++){
           if (mid[k] > mid[k+1]){
               bigger = big(mid[k],mid[k+1]);
               smaller = small(mid[k],mid[k+1]);
               mid[k] = smaller;
               mid[k+1] = bigger;
               }
          }
      }
      if (N%2!=1){//N이 짝수인경우  
        median = (mid[middle] + mid[middle+1])/2;
      }
      else{//N이 홀수인 경우
        median = mid[middle];
      }   
  }     

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(median);
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
  count+=1;
}
// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}

float big(float a,float b){
  if (a>b)
    return a;
  else
    return b;
    }

float small(float a,float b){
  if (a<b)
    return a;
  else
    return b;
    }

 
