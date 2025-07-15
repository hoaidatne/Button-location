#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Định nghĩa các hằng số cho chân phần cứng và màn hình OLED
#define BUTTON_PIN 34  // Chân GPIO34 dùng cho nút SOS
#define SCREEN_WIDTH 128  // Chiều rộng màn hình OLED (pixel)
#define SCREEN_HEIGHT 32  // Chiều cao màn hình OLED (pixel)
#define OLED_RESET -1  // Chân reset cho OLED (không sử dụng)
#define SCREEN_ADDRESS 0x3C // Địa chỉ I2C của màn hình SSD1306

// Định nghĩa các hằng số cho giao tiếp với module SIM
#define simSerial SIM  // Bí danh cho giao tiếp serial với SIM
#define MCU_SIM_BAUDRATE 9600  // Tốc độ baud cho module SIM
#define MCU_SIM_TX_PIN 5  // Chân TX cho module SIM
#define MCU_SIM_RX_PIN 4  // Chân RX cho module SIM
#define PHONE_NUMBER "+84392607120"  // Số điện thoại nhận SMS

// Định nghĩa các chân cho giao tiếp với module LoRa
#define RXD2 16  // Chân RX của ESP32 nối với TX của LoRa
#define TXD2 17  // Chân TX của ESP32 nối với RX của LoRa
#define M0 19  // Chân M0 của LoRa để cấu hình chế độ
#define M1 18  // Chân M1 của LoRa để cấu hình chế độ
#define BUZZER 25  // Chân điều khiển còi báo SOS

// Khởi tạo đối tượng màn hình OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Khởi tạo đối tượng serial cho LoRa và SIM
HardwareSerial LoRa(2);  // UART2 dùng để giao tiếp với LoRa
HardwareSerial SIM(1);   // UART1 dùng để giao tiếp với SIM

// Các biến để quản lý thời gian và trạng thái SOS
signed long lastReceivedTime = -60000;  // Thời gian nhận tín hiệu SOS cuối cùng
const unsigned long sosTimeout = 60000; // Thời gian chờ 60 giây để thoát SOS
signed long lastSMSSentTime = -45000;  // Thời gian gửi SMS cuối cùng
const unsigned long smsInterval = 45000; // Khoảng cách 45 giây giữa các SMS
bool sosActive = false;  // Cờ theo dõi trạng thái SOS

// Hàm lấy thời gian hiện tại từ module SIM
String getSimTime() {
  while (simSerial.available()) simSerial.read(); // Xóa bộ đệm serial

  simSerial.println("AT+CCLK?");  // Gửi lệnh AT để lấy thời gian
  unsigned long timeout = millis();
  String response = "";

  // Chờ phản hồi trong 3 giây
  while (millis() - timeout < 3000) {
    while (simSerial.available()) {
      char c = simSerial.read();
      response += c;  // Xây dựng chuỗi phản hồi
    }

    // Kiểm tra nếu phản hồi chứa dữ liệu thời gian
    if (response.indexOf("+CCLK:") >= 0) {
      Serial.println("Phản hồi đầy đủ: " + response);
      int start = response.indexOf("\"") + 1;
      int end = response.lastIndexOf("\"");
      if (start > 0 && end > start) {
        String time = response.substring(start, end);  // Trích xuất thời gian
        Serial.println("Thời gian trích xuất: " + time);
        return time;
      }
    }
  }

  Serial.println("⚠️ Không lấy được thời gian từ SIM");
  return "unknown";  // Trả về giá trị mặc định nếu thất bại
}

// Hàm chờ và đọc phản hồi từ module SIM
void sim_at_wait() {
  delay(100);  // Đợi 100ms để nhận phản hồi
  while (simSerial.available()) {
    Serial.write(simSerial.read());  // In phản hồi ra Serial monitor
  }
}

// Hàm gửi lệnh AT tới module SIM
bool sim_at_cmd(String cmd) {
  simSerial.println(cmd);  // Gửi lệnh AT
  sim_at_wait();  // Chờ và đọc phản hồi
  return true;  // Luôn trả về true (không kiểm tra lỗi)
}

// Hàm gửi một ký tự tới module SIM
bool sim_at_send(char c) {
  simSerial.write(c);  // Gửi ký tự
  return true;  // Luôn trả về true
}

// Hàm gửi tin nhắn SMS chứa thông điệp SOS
void sent_sms(String sos) {
  sim_at_cmd("AT+CMGF=1");  // Đặt chế độ SMS là văn bản
  String temp = "AT+CMGS=\"";
  temp += (String)PHONE_NUMBER;
  temp += "\"";  // Đặt số điện thoại nhận SMS
  sim_at_cmd(temp);  // Gửi lệnh để bắt đầu SMS
  sim_at_cmd(sos);  // Gửi nội dung tin nhắn SOS
  sim_at_send(0x1A);  // Gửi ký tự kết thúc SMS (Ctrl+Z)
}

// Hàm gửi dữ liệu SOS lên Firebase qua HTTP POST
void sendToFirebase(String sosData) {
  String timestamp = getSimTime();  // Lấy thời gian từ SIM
  String firebaseHost = "https://e32-tracker-default-rtdb.firebaseio.com/sos.json";
  String jsonPayload = "{\"message\":\"" + sosData + "\",\"timestamp\":\"" + timestamp + "\"}";  // Tạo payload JSON

  sim_at_cmd("AT+HTTPTERM");  // Kết thúc phiên HTTP trước đó
  delay(100);
  
  sim_at_cmd("AT+HTTPINIT");  // Khởi tạo phiên HTTP
  delay(100);

  sim_at_cmd("AT+HTTPPARA=\"CID\",1");  // Đặt channel ID cho HTTP
  delay(100);

  sim_at_cmd("AT+HTTPPARA=\"URL\",\"" + firebaseHost + "\"");  // Đặt URL đích
  delay(100);

  sim_at_cmd("AT+HTTPPARA=\"CONTENT\",\"application/json\"");  // Đặt loại nội dung là JSON
  delay(100);

  simSerial.println("AT+HTTPDATA=" + String(jsonPayload.length()) + ",10000");  // Thông báo độ dài dữ liệu

  // Chờ phản hồi "DATA" từ SIM trước khi gửi dữ liệu
  unsigned long timeout = millis();
  while (!simSerial.find("DATA")) {
    if (millis() - timeout > 3000) {
      Serial.println("❌ Không nhận được phản hồi DATA");
      return;
    }
  }

  simSerial.print(jsonPayload);  // Gửi nội dung JSON
  delay(1000);

  sim_at_cmd("AT+HTTPACTION=1");  // Thực hiện lệnh POST
  delay(5000);  // Đợi module gửi xong

  // Đọc phản hồi từ server (+HTTPACTION:1,200,<datalen>)
  while (simSerial.available()) {
    String line = simSerial.readStringUntil('\n');
    if (line.indexOf("+HTTPACTION:") >= 0) {
      Serial.println("➡️ Phản hồi: " + line);
    }
  }

  sim_at_cmd("AT+HTTPREAD");  // Đọc nội dung trả về từ server
  delay(500);

  sim_at_cmd("AT+HTTPTERM");  // Kết thúc phiên HTTP
}

// Hàm thiết lập ban đầu
void setup() {
  delay(20);  // Đợi 20ms để ổn định
  Serial.begin(115200);  // Khởi động Serial để debug
  pinMode(BUTTON_PIN, INPUT);  // Cấu hình chân nút SOS là input
  LoRa.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Khởi động UART2 cho LoRa
  pinMode(M0, OUTPUT);  // Cấu hình chân M0 của LoRa
  pinMode(M1, OUTPUT);  // Cấu hình chân M1 của LoRa
  digitalWrite(M0, LOW);  // Đặt LoRa ở chế độ Normal
  digitalWrite(M1, LOW);
  pinMode(BUZZER, OUTPUT);  // Cấu hình chân còi
  digitalWrite(BUZZER, LOW);  // Tắt còi ban đầu
  Serial.println("🚀 Module LoRa E32-433T20D đang khởi động...");
  delay(1000);

  // Khởi động màn hình OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 không tìm thấy! Kiểm tra kết nối."));
  }
  display.clearDisplay();  // Xóa màn hình
  display.setTextSize(1);  // Đặt kích thước chữ
  display.setTextColor(SSD1306_WHITE);  // Đặt màu chữ trắng

  display.clearDisplay(); 
  display.setCursor(0, 0); 
  display.print("loanding...");  // Hiển thị thông báo khởi động
  display.display(); 

  // Thiết lập module SIM
  Serial.println("\n\n\n\n-----------------------\nSystem started!!!!");
  delay(8000);  // Đợi 8 giây để cấp nguồn ổn định
  SIM.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);  // Khởi động UART1 cho SIM
  sim_at_cmd("AT");  // Kiểm tra lệnh AT
  sim_at_cmd("ATI");  // Lấy thông tin sản phẩm
  sim_at_cmd("AT+CPIN?");  // Kiểm tra khe SIM
  sim_at_cmd("AT+CSQ");  // Kiểm tra chất lượng tín hiệu
  sim_at_cmd("AT+CIMI");  // Lấy mã IMSI của SIM

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("NO SIGNAL SOS");  // Hiển thị trạng thái ban đầu
  display.display();
}

// Hàm vòng lặp chính
void loop() {
  // Nhận dữ liệu từ LoRa
  if (LoRa.available()) {
    String received = LoRa.readStringUntil('\n');  // Đọc dữ liệu tới ký tự xuống dòng
    received.trim();  // Xóa khoảng trắng

    if (received.startsWith("SOS")) {  // Kiểm tra tín hiệu SOS
      lastReceivedTime = millis();  // Cập nhật thời gian nhận SOS
      sosActive = true;  // Đặt trạng thái SOS

      // Nếu đủ 45 giây kể từ SMS trước
      if (millis() - lastSMSSentTime >= smsInterval) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("SIGNAL SOS!!!");  // Hiển thị thông báo SOS
        display.display();

        Serial.println(received);  // In dữ liệu nhận được
        digitalWrite(BUZZER, HIGH);  // Bật còi báo động
        sent_sms(received);  // Gửi SMS
        sendToFirebase(received);  // Gửi dữ liệu lên Firebase
        lastSMSSentTime = millis();  // Cập nhật thời gian gửi SMS
      }
    }
  }

  // Nếu vượt quá thời gian chờ và đang ở trạng thái SOS
  if (millis() - lastReceivedTime > sosTimeout && sosActive) {
    digitalWrite(BUZZER, LOW);  // Tắt còi

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("NO SIGNAL SOS");  // Hiển thị trạng thái không SOS
    display.display();

    sosActive = false;  // Kết thúc trạng thái SOS
  }

  delay(100);  // Đợi 100ms trước khi lặp lại
}
