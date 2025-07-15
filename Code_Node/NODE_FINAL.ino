#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Khai báo chân và cấu hình cho các thiết bị
#define BUTTON_PIN 34  // GPIO34 cho nút nhấn SOS
#define SCREEN_WIDTH 128  // Độ rộng màn hình OLED (128 pixel)
#define SCREEN_HEIGHT 32  // Chiều cao màn hình OLED (32 pixel)
#define OLED_RESET -1  // Chân reset OLED (không sử dụng)
#define SCREEN_ADDRESS 0x3C  // Địa chỉ I2C của màn hình OLED SSD1306
#define RXD2 16  // Chân RX UART2 của ESP32 nối với TX của LoRa
#define TXD2 17  // Chân TX UART2 của ESP32 nối với RX của LoRa
#define M0 19  // Chân điều khiển chế độ LoRa
#define M1 18  // Chân điều khiển chế độ LoRa
#define BUZZER 25  // Chân điều khiển LED/Buzzer báo SOS

// Khởi tạo đối tượng OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Cấu hình UART cho GPS
static const int RXPin = 4, TXPin = 5;  // Chân UART2 cho GPS
static const uint32_t GPSBaud = 9600;  // Tốc độ baud GPS

// Khởi tạo đối tượng GPS và UART
TinyGPSPlus gps;  // Đối tượng xử lý dữ liệu GPS
HardwareSerial mySerial(1);  // UART2 cho GPS
HardwareSerial LoRa(2);  // UART2 cho LoRa

// Khai báo biến toàn cục
bool SOS = false;  // Trạng thái SOS (true: kích hoạt, false: bình thường)
unsigned long previousMillis = 0;  // Thời điểm lần gửi tín hiệu trước
const long interval = 10000;  // Khoảng thời gian gửi tín hiệu SOS (10s)
float Latitude, Longitude;  // Lưu tọa độ GPS dạng số thực
String LatitudeString, LongitudeString, s;  // Chuỗi tọa độ và dữ liệu gửi LoRa
bool lastSOSState = false;  // Lưu trạng thái SOS trước đó

void setup() {
    Serial.begin(115200);  // Khởi tạo Serial để debug
    pinMode(BUTTON_PIN, INPUT);  // Cấu hình nút nhấn SOS là input
    mySerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);  // Khởi tạo UART cho GPS
    LoRa.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Khởi tạo UART cho LoRa
    pinMode(M0, OUTPUT);  // Cấu hình chân M0 LoRa
    pinMode(M1, OUTPUT);  // Cấu hình chân M1 LoRa
    digitalWrite(M0, LOW);  // Đặt LoRa ở chế độ Normal
    digitalWrite(M1, LOW);  // Đặt LoRa ở chế độ Normal
    pinMode(BUZZER, OUTPUT);  // Cấu hình chân LED/Buzzer
    digitalWrite(BUZZER, LOW);  // Tắt LED/Buzzer ban đầu
    Serial.println("🚀 Module LoRa E32-433T20D đang khởi động...");
    delay(1000);  // Đợi 1s để ổn định

    // Khởi động màn hình OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 không tìm thấy! Kiểm tra kết nối."));
    }
    display.clearDisplay();  // Xóa màn hình OLED
    display.setTextSize(1);  // Cỡ chữ nhỏ
    display.setTextColor(SSD1306_WHITE);  // Màu chữ trắng
}

void loop() {
    KT_NUT_NHAN_S0S();  // Kiểm tra trạng thái nút nhấn SOS
    KT_TRANG_THAI();  // Kiểm tra và xử lý trạng thái SOS/bình thường
    HIENTHI_TREN_OLED();  // Cập nhật hiển thị thời gian và trạng thái trên OLED
}

void KT_NUT_NHAN_S0S() {
    // Kiểm tra nút nhấn SOS
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50);  // Chống nhiễu nút nhấn
        if (digitalRead(BUTTON_PIN) == LOW) {
            while (digitalRead(BUTTON_PIN) == LOW) {}  // Đợi thả nút
            SOS = !SOS;  // Đổi trạng thái SOS (bật/tắt)
        }
    }
}

void KT_TRANG_THAI() {
    if (SOS == false) {
        Serial.println("BINH THUONG");  // Trạng thái bình thường
        digitalWrite(BUZZER, LOW);  // Tắt LED/Buzzer
    } else {
        Serial.println("TRANG THAI SOS");  // Trạng thái SOS
        TRANG_THAI_SOS();  // Gửi tín hiệu SOS
        digitalWrite(BUZZER, HIGH);  // Bật LED/Buzzer
    }
}

void HIENTHI_TREN_OLED() {
    // Nhận và xử lý dữ liệu từ GPS
    while (mySerial.available()) {
        gps.encode(mySerial.read());  // Đọc và giải mã dữ liệu GPS
    }
    // Lưu thời gian GPS gần nhất
    static int lastHour, lastMinute, lastSecond;
    if (gps.time.isValid()) {
        lastHour = gps.time.hour();
        lastMinute = gps.time.minute();
        lastSecond = gps.time.second();
    }

    // Hiển thị trên OLED
    display.clearDisplay();  // Xóa màn hình
    display.setTextSize(1);  // Cỡ chữ
    display.setTextColor(SSD1306_WHITE);  // Màu chữ trắng

    // Hiển thị thời gian (UTC+7)
    display.setCursor(0, 0);
    display.print("Time: ");
    display.print(lastHour + 7);  // Điều chỉnh múi giờ
    display.print(":");
    if (lastMinute < 10) display.print("0");
    display.print(lastMinute);
    display.print(":");
    if (lastSecond < 10) display.print("0");
    display.print(lastSecond);

    // Hiển thị trạng thái
    display.setCursor(0, 10);
    if (SOS) {
        display.print("DANG GUI TIN HIEU SOS...");  // Thông báo trạng thái SOS
    } else {
        display.print("CLICK BUTTON ON SOS");  // Thông báo trạng thái bình thường
    }
    display.display();  // Cập nhật màn hình
}

void TRANG_THAI_SOS() {
    // Nhận và xử lý dữ liệu từ GPS
    while (mySerial.available()) {
      display.print("Dang cap nhat GPS: ");
        gps.encode(mySerial.read());  // Đọc và giải mã dữ liệu GPS
    }
    // Gửi tọa độ qua LoRa mỗi 10s
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) 
       {
        if(gps.location.isValid())
        {
        previousMillis = currentMillis;
        Serial.println("📡 Gửi tọa độ qua LoRa:");
        Latitude = gps.location.lat();  // Lấy vĩ độ
        LatitudeString = String(Latitude, 6);  // Chuyển vĩ độ thành chuỗi
        Longitude = gps.location.lng();  // Lấy kinh độ
        LongitudeString = String(Longitude, 6);  // Chuyển kinh độ thành chuỗi
        s = "SOS1:https://www.google.com/maps?q=";  // Tạo chuỗi định dạng link Google Maps
        s += LatitudeString + "," + LongitudeString + "\n";
        LoRa.println(s);  // Gửi dữ liệu qua LoRa
        Serial.println("📡 da gui toa do.");
        }
        else
        {
        LoRa.println("SOS1:khong nhan duoc toa do");  // Gửi dữ liệu qua LoRa
        Serial.println("khong nhan duoc toa do");          
        }
        
    }
}