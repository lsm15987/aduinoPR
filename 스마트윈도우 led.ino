#include <LiquidCrystal_I2C.h>

#include <SoftwareSerial.h>
SoftwareSerial soft(12, 13);


#include <Wire.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

int led = 0;

// 모터 관련 설정
#define STEP_DELAY 3000
#define STEP_COUNT 200
#define ENABLE_PIN 6  //모터의 전압 차단유무
#define DIR_PIN 5     //회전의 방형
#define STEP_PIN 4    //회전의 정도
#define LIMIT_PIN1 7  //닫히는 스위치
#define LIMIT_PIN2 8  //열리는 스위치

int window_control = 1;  // 0: 수동 , 1:자동 ,작동하는지 확인을 위해 1로 지정
int window_status = 0;   // 0 : 정지 , 1 : 열리도록 , 2 : 닫히도록

int prev_window_status = 0;      // 창문의 마지막 상태를 저장, 0:알수 없음, 1:열림, 2:닫힘
unsigned long last_step_micros;  // 마지막으로 모터를 움직인 시간을 저장
int last_step_pin_status;
int prev_limit_1_status;  //이벤트 발생 전 닫히는 스위치의 상태
int prev_limit_2_status;  //이벤트 발생 전 열리는 스위치의 상태

#define SENSOR_CHECK_PERIOD 2000

//온도센서 설정
#include "DHT.h"
#define DHTPIN 11  // 11핀에다가 온도 센서 연결
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
int8_t humi, temp;  // humi, temp 변수 선언
unsigned long dht_prev_readed_millis;


#define LIMIT_TEMP_MAX_VALUE 30  //최고 온도, 이상이면 더움 감지
#define LIMIT_TEMP_MIN_VALUE 11  //최저 온도 , 이하이면 추움 감지

// 빗물 감지 센서
#define RAIN_SENSOR_PIN A0    // A0핀에다가 빗물 센서 연결
#define LIMIT_RAIN_VALUE 700  // 비가오는지 구분할 센서값 , 미만이면 비 감지
int rain_value = 0;

// 가스 감지 센서
#define GAS_SENSOR_PIN A1    // A1핀에다가 가스 센서 연결
#define LIMIT_GAS_VALUE 500  //이상이면 가스 감지
int gas_value = 0;

// 미세먼지 센서
#define LIMIT_DUST_VALUE 150  // 미세먼지 나쁨 감지
int DUST_SENSOR_PIN = A2;     // 미세먼지 핀 번호
int DUST_LED_PIN = 12;        // 미세먼지 센서 안에 있는 적외선 led 핀 번호
float dust_value = 0;         // 센서에서 입력 받은 미세먼지 값
float dustDensityug = 0;      // ug/m^3 값을 계산

int sampling = 280;  // 적외선 led를 키고, 센서 값을 읽어 들여 미세먼지를 측정하는 샘플링 시간
int waiting = 40;
float stop_time = 9680;  // 센서를 구동하지 않는 시간

void setup() {
  lcd.init();  //LCD 사용 시작    lcd.begin() 으로 하니까 에러 남
  lcd.backlight();

  Serial.begin(9600);
  soft.begin(9600);

  // 모터 관련 코드
  pinMode(DIR_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // 스위치 관련 코드
  pinMode(LIMIT_PIN1, INPUT);
  pinMode(LIMIT_PIN2, INPUT);

  // 모터 관련 코드
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIR_PIN, HIGH);
  last_step_micros = millis();

  // 스위치 관련 코드
  prev_limit_1_status = digitalRead(LIMIT_PIN1);
  prev_limit_2_status = digitalRead(LIMIT_PIN2);
  window_status = 0;

  // 온도 센서 초기화
  Serial.println("DHT SENSOR INIT");
  dht.begin();

  // 미세먼지 적외선 led를 출력으로 설정
  pinMode(DUST_LED_PIN, OUTPUT);
}

void loop() {
  // 센서 읽기
  // 온도 센서 측정

  if (((millis() - dht_prev_readed_millis) > SENSOR_CHECK_PERIOD) && (window_status == 0))  // 창문이 닫힌 상태이면서 2초 이상마다 정보를 출력
  {
    humi = (int8_t)dht.readHumidity();
    temp = (int8_t)dht.readTemperature();
    digitalWrite(DUST_LED_PIN, LOW);    // 미세먼지 적외선
    delayMicroseconds(sampling);        // 샘플링해주는 시간.
    dht_prev_readed_millis = millis();  //실행 했던 시간 저장

    dust_value = analogRead(DUST_SENSOR_PIN);
    delay(200);
    // 빗물 감지 센서
    rain_value = analogRead(RAIN_SENSOR_PIN);

    // 가스 감지 센서
    gas_value = analogRead(GAS_SENSOR_PIN);

    // 센서 값 읽어오기


    digitalWrite(DUST_LED_PIN, HIGH);  // LED 끄기
    delayMicroseconds(stop_time);      // LED 끄고 대기

    // 센서값 시리얼 모니터 출력
    Serial.println("###############################");
    Serial.println("######  SENSOR VALUE ##########");
    Serial.println("###############################");

    // 온습도 센서 값 출력
    Serial.print("TEMPERATURE : ");
    Serial.print(temp);
    Serial.println(" DEG");

    Serial.print("HUMIDITY : ");
    Serial.print(humi);
    Serial.println(" %");

    // 미세먼지 센서 값 출력
    dustDensityug = (0.17 * (dust_value * (5.0 / 1024)) - 0.1) * 1000;  // 미세먼지 값 계산
    Serial.print("Dust Density [ug/m3]: ");                             // 시리얼 모니터에 미세먼지 값 출력
    Serial.print(dustDensityug);

    if (dustDensityug < LIMIT_DUST_VALUE) {
      Serial.println("(NOT DETECTED)");
    } else {
      Serial.println("(DETECTED)");
    }

    lcd.setCursor(0, 0);
    lcd.print("dust:");
    lcd.print(dustDensityug);
    lcd.setCursor(0, 1);
    lcd.print("gas:");
    lcd.print(gas_value);
    delay(500);
    lcd.clear();
    // 빗물 센서 값 출력
    Serial.print("RAIN : ");
    Serial.print(rain_value);

    if (rain_value < LIMIT_RAIN_VALUE) {
      Serial.println("(DETECTED)");
    } else {
      Serial.println("(NOT DETECTED)");
    }

    // 가스 센서 값 출력
    Serial.print("GAS : ");
    Serial.print(gas_value);
    if (gas_value > LIMIT_GAS_VALUE) {
      Serial.println("(DETECTED)");
    } else {
      Serial.println("(NOT DETECTED)");
    }

    // 센서값 블루투스 전송
    soft.write(20);
    soft.write(temp);
    soft.write(humi);
    soft.write(rain_value);
    soft.write(rain_value >> 8);
    soft.write(gas_value);
    soft.write(gas_value >> 8);
    soft.write((uint8_t)0);
    soft.write((uint8_t)0);
    soft.write(21);
  }

  if (window_control == 1) {
    if (window_control == 1)  // 자동 제어 모드인 경우
    {
      if ((gas_value >= LIMIT_GAS_VALUE))  // 가스 값이 크고 창문이 열린 상태가 아니면
      {
        if (prev_window_status != 1) {
          window_status = 1;  // 창문 열기
        }
      } else if ((rain_value < LIMIT_RAIN_VALUE))  // 빗물 센서 값이 크고 창문이 닫힌 상태가 아니면
      {
        if (prev_window_status != 2) {
          window_status = 2;  // 창문 닫기
        }
      } else if ((temp >= LIMIT_TEMP_MAX_VALUE))  // 온도가 높고 창문이 열린 상태가 아니면
      {
        if (prev_window_status != 1) {
          window_status = 1;  // 창문 열기
        }
      } else if ((temp <= LIMIT_TEMP_MIN_VALUE))  // 온도가 낮고 창문이 닫힌 상태가 아니면
      {
        if (prev_window_status != 2) {
          window_status = 2;  // 창문 닫기
        }
      } else if ((dustDensityug >= LIMIT_DUST_VALUE))  // 미세먼지가 나쁘고 창문이 닫힌 상태가 아니면
      {
        if (prev_window_status != 2) {
          window_status = 2;  // 창문닫기
        }
      } else {
        window_status = 0;  // 윗 사항이 아닌 경우 창문 정지
      }
    }
  }

  /*  while ( soft.available() >= 2) 
  {
    int ch = soft.read();
    Serial.write(ch);
    if ( ch == 20 )
    {
      delay(50);
      if ( soft.available() )
      {
        ch = soft.read();
        Serial.print(ch);
        Serial.print(" ");

        if ( ch == '0' ) // 0 을 수신하면 창문 이동 정지
        {
          window_status = 0;
          window_control = 0; // 창문 제어 명령이 수신된 경우 수동 제어로 전환
          digitalWrite(ENABLE_PIN, HIGH);
          prev_window_status = 0; // 마지막 창문의 상태는 알수 없음
        }
        if ( ch == '1' ) // 1을 수신하면 창문 열기
        {
          window_status = 1;
          window_control = 0; // 창문 제어 명령이 수신된 경우 수동 제어로 전환
          prev_window_status = 0; // 마지막 창문의 상태는 알수 없음
        }
        if ( ch == '2' ) // 2를 수신하면 창문 닫기
        {
          window_status = 2;
          window_control = 0; // 창문 제어 명령이 수신된 경우 수동 제어로 전환
          prev_window_status = 0; // 마지막 창문의 상태는 알수 없음
        }
        if ( ch == '3' ) // 3을 수신하면 창문 제어 자동
        {
          window_control = 1;
          prev_window_status = 0; // 마지막 창문의 상태는 알수 없음
        }
        if ( ch == '4' ) // 4를 수신하면 창문 제어 수동
        {
          window_control = 0;
          prev_window_status = 0; // 마지막 창문의 상태는 알수 없음
        }
      }
    }
  }*/
  if (window_control == 1)  // 자동 제어 모드인 경우
  {
    if ((gas_value >= LIMIT_GAS_VALUE))  // 가스 값이 크고 창문이 열린 상태가 아니면
    {
      if (prev_window_status != 1) {
        window_status = 1;  // 창문 열기
      }
    }
    /*else if ( ( rain_value < LIMIT_RAIN_VALUE ) ) // 빗물 센서 값이 크고 창문이 닫힌 상태가 아니면
    {
      if ( prev_window_status != 2 )
      {
        window_status = 2; // 창문 닫기
      }
    }*/
    else if ((temp >= LIMIT_TEMP_MAX_VALUE))  // 온도가 높고 창문이 열린 상태가 아니면
    {
      if (prev_window_status != 1) {
        window_status = 1;  // 창문 열기
      }
    } else if ((temp <= LIMIT_TEMP_MIN_VALUE))  // 온도가 낮고 창문이 닫힌 상태가 아니면
    {
      if (prev_window_status != 2) {
        window_status = 2;  // 창문 닫기
      }
    } else if ((dustDensityug >= LIMIT_DUST_VALUE))  // 미세먼지가 나쁘고 창문이 닫힌 상태가 아니면
    {
      if (prev_window_status != 2) {
        window_status = 2;  // 창문닫기
      }
    } else {
      window_status = 0;  // 윗 사항이 아닌 경우 창문 정지
    }
  }

  if (window_status == 1) {

    if (led == 0) {
      if ((micros() - last_step_micros) > STEP_DELAY)  //돌린지 3초가 지났다면
      {


        digitalWrite(ENABLE_PIN, LOW);  //전력공급
        delayMicroseconds(STEP_DELAY);
        digitalWrite(DIR_PIN, HIGH);  //열림 방향
        delayMicroseconds(STEP_DELAY);

        last_step_pin_status = !last_step_pin_status;  //변수 초기화를 안하여 기본값 0
        digitalWrite(STEP_PIN, last_step_pin_status);
        last_step_micros = micros();
        int limit_status = digitalRead(LIMIT_PIN1);
        if (limit_status == LOW)  // LIMIT 스위치가 눌러진 경우
        {
          //window_status = 0; // 대기 상태로 변경
          digitalWrite(ENABLE_PIN, HIGH);  //전력 차단
          //prev_window_status = 1; // 마지막 창문의 상태는 열림 상태
          led = 1;
        }
      }
    }
    if (led == 1) {
      lcd.setCursor(0, 0);
      lcd.print("dust: ");
      lcd.print(dustDensityug);
      lcd.setCursor(0, 1);
      lcd.print("open the door");
      delay(2000);
      lcd.clear();
      window_status = 0;       // 대기 상태로 변경
      prev_window_status = 1;  // 마지막 창문의 상태는 열림 상태
      led = 0;
    }
  }

  if (window_status == 2) {
    if (led == 0) {
      if ((micros() - last_step_micros) > STEP_DELAY) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(DIR_PIN, LOW);
        last_step_pin_status = !last_step_pin_status;
        digitalWrite(STEP_PIN, last_step_pin_status);
        last_step_micros = micros();
        int limit_status = digitalRead(LIMIT_PIN2);
        if (limit_status == LOW)  // LIMIT 스위치가 눌러진 경우
        {
          //window_status = 0; // 대기 상태로 변경
          digitalWrite(ENABLE_PIN, HIGH);
          //prev_window_status = 2; // 마지막 창문의 상태는 닫힘 상태
          led=1;
        }
      }
    }
    if (led == 1) {
      lcd.setCursor(0, 0);
      lcd.print("dust: ");
      lcd.print(dustDensityug);
      lcd.setCursor(0, 1);
      lcd.print("close the door");
      delay(2000);
      lcd.clear();
      window_status = 0;       // 대기 상태로 변경
      prev_window_status = 2;  // 마지막 창문의 상태는 열림 상태
      led = 0;
    }
  }
}