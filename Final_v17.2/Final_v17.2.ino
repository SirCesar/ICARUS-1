// Equipe: ICARUS, código do satélite: ICARUS-1
// Autores: Adriano César de Sousa Pereira, Kayque Batista de Oliveira, João Pedro Bineli Alves, Kevin Cohin Hereda de Freitas Marinho
// Instituição responsável: IFSP - Campus São João da Boa Vista, UNESP - Campus São João da Boa Vista
// 
#include <WiFi.h>              // biblioteca do módulo wi-fi
#include <HTTPClient.h>        // biblioteca de cliente http
#include <TinyGPSPlus.h>       // biblioteca do GPS
#include <Wire.h>              // biblioteca de endereçamento
#include <SPI.h>               // biblioteca de comunicação SPI
#include <Adafruit_BMP280.h>   // biblioteca do BMP pela Adafruit
#include <Adafruit_MPU6050.h>  // Biblioteca do MPU6050 pela Adafruit
#include <Adafruit_Sensor.h>   // Biblioteca de complemento para MPU6050 pela Adafruit
#include <FS.h>                // biblioteca do leitor de cartão SD
#include <SD.h>                // biblioteca do leitor de cartão SD
#include <esp_sleep.h>         // biblioteca do modo hibernar do ESP32, deep-sleep
// DADOS DE CONEXÃO WI-FI, LOCAL:
/*
String serverName = "https://obsat.org.br/teste_post/envio.php";      // servidor de testes da olimpíada
char *ssid = "Nome da rede";                                          // nome da rede wi-fi local
char *password = "Senha da rede";                                     // senha da rede wi-fi localW
*/
// PÁGINA WEB COM RECEPÇÃO DOS DADOS ENVIADOS:
// https://obsat.org.br/teste_post/index.php                          // use este link para ver os dados enviados
unsigned long t1 = 0;                // tempo inicial de tentativa de conexão wi-fi
unsigned long t2 = 0;                // tempo final de tentativa de conexão wi-fi
unsigned long t3 = 0;                // calcula o tempo restante do ciclo (limite de 10s)
unsigned long t4 = 0;                // tempo inicial de envio dos dados
RTC_DATA_ATTR unsigned long t5 = 0;  // tempo final de envio dos dados - inicial
unsigned long t6 = 0;
// DADOS DE CONEXÃO WI-FI, LOCAL DO EVENTO:
/*
String serverName = "http://192.168.0.1/";
char *ssid = "OBSAT_WIFI";
char *password = "OBSatZenith1000";
*/
/* TEMPORIZADORES t0, t1, t2, t3, t4, t5:
  É preciso ser feito ajuste de erro nos tempos para garantir que os dados sejam enviados a cada 4 minutos e 0 segundos, nem mais e nem menos.
  A fim de atingir tal propósito, é feita a leitura do tempo até se conectar ao wi-fi (necessário atraso) e o tempo para enviar os dados (próximo de 1,2s)
  Subtraindo esses tempos "consumidos" de um total de 9800ms, pois o ciclo de envio tem um intervalo livre de 10000ms, sendo 10000 o momento de envio,
  mas é importante evitar que o limite do tempo seja 10000ms, a fim de garantir que não exceda os 10s dentro do ciclo e trave o processo.
*/
// -------------------------------------------------------------------
// INSTANCIANDO O PAYLOAD:
#define S0 32                                               // GPIO 32 (usando ADC)
#define S1 33                                               // GPIO 33 (usando ADC)
#define S2 25                                               // GPIO 25 (usando ADC)
#define S3 26                                               // GPIO 26 (usando ADC)
const int analog = 34;                                      // sig = GPI O4
unsigned myAnalogRead(short inputCH, short an_in);          // função para leitura do MUX 16
#define canais 7                                            // número de pinos usados no MUX
int cont_canais = 0;                                        // contador de canais usados
float valor[canais];                                        // matriz com tamanho de canais, ou seja 7
const int pinos_canais[canais] = { 0, 1, 2, 3, 4, 5, 15 };  // divisor de tensão, pinos do mux
// Dados de calibração (cada item refere-se a um termistor):
const float A[canais] = { 7.080, 8.397, 5.565, 15.464, 5.969, 5.909 };
const float B[canais] = { -45.291, -57.141, -33.678, -120.117, -34.059, -34.854 };
const float C[canais] = { 104.323, 141.606, 73.709, 339.504, 68.471, 72.845 };
const float D[canais] = { -72.765, -121.328, -42.582, -370.290, -27.292, -33.766 };
const float E[canais] = { 11.649, 33.488, 2.146, 128.043, -4.785, -3.214 };
RTC_DATA_ATTR float valorT[6];       // valorT armazena leitura analógica de 0-6 termistores
RTC_DATA_ATTR double temp;           // variável com valores positivos e negativos
RTC_DATA_ATTR int nivelBateria = 0;  // armazena percentual de carga da bateria
// -------------------------------------------------------------------
// INSTANCIANDO O MÓDULO SD:
String s_conteudo = "";
char const *char_conteudo = s_conteudo.c_str();
RTC_DATA_ATTR unsigned long t0 = 0;          // tempo de leitura e escrita de dados
RTC_DATA_ATTR unsigned long contador_0 = 0;  // contador de escrita no cartão
// -------------------------------------------------------------------
// INSTANCIANDO O ACELERÔMETRO/GIROSCÓPIO:
Adafruit_MPU6050 mpu;                                  // renomeia a bliblioteca Adafruit_MPU6050 para mpu, a fim de simplificação quando chamada
RTC_DATA_ATTR float Gx, Gy, Gz, Ax, Ay, Az, mpu_temp;  // Eixos e temperatura
RTC_DATA_ATTR String mpu_gyro = "[0.00,0.00,0.00]", mpu_acel = "[0.00,0.00,0.00]";
// -------------------------------------------------------------------
// INSTANCIANDO O BMP280:
#define BMP_SCK (13)                              // pino SCK do módulo
#define BMP_MISO (12)                             // pino MISO do módulo
#define BMP_MOSI (11)                             // pino MOSI do módulo
#define BMP_CS (10)                               // pino CS do módulo
Adafruit_BMP280 bmp;                              // renomeia a biblioteca Adafruit_BMP280 para bmp, a fim de simplificação quando chamada
RTC_DATA_ATTR float bmp_temp, bmp_pres, bmp_alt;  // bmp_temp = temperatura do bmp280; bmp_pres = pressão do bmp280; bmp_alt = altitude do bmp280
// -------------------------------------------------------------------
// INSTANCIANDO O GPS:
#define RXD2 16  // RXD2=GPIO16, RX do ESP32 com TX do GPS
#define TXD2 17  // TXD2=GPIO17, TX do ESP32 com RX do GPS
HardwareSerial neogps(1);
TinyGPSPlus gps;                                                                     // renomeia a biblioteca TinyGPSPlus para gps, a fim de simplificação quando chamada
RTC_DATA_ATTR float ALTURA_GPS = 0.0, LAT_GPS = 0.0, LGN_GPS = 0.0, KMPH_GPS = 0.0;  // LAT_GPS = Latitude do GPS; LGN_GPS = Longitude do GPS; KMPH = velocidade em km/h do GPS
RTC_DATA_ATTR int SAT_GPS = 0;                                                       // SAT_GPS = quantidade de satélites conectados ao módulo GPS, a fim de triangulação do sinal (mínimo 4)
RTC_DATA_ATTR float gps_pres = 0.0;                                                  // variável que irá armazenar o cálculo de pressão com altura do GPS (ALTURA_GPS)
RTC_DATA_ATTR unsigned long intervalo_gps = 1000;
// MODO DEEP-SLEEP:
/*
  O modo de operação Deep-Sleep faz com que o microcontrolador reinicie de acordo com acionamento externo (botão) ou temporização, como é o caso atual do projeto
  o ESP32 irá acordar a cada 10 segundos, já com tempo corrigido dado o processo de leitura + escrita no cartão. Não há loop(), o processo fica no setup()
  além do exposto, é preciso atribuir RTC_DATA_ATTR nas variáveis onde é necessário memorizar os valores, pois quando reinicia, apaga a memória volátil.
*/
RTC_DATA_ATTR unsigned long boot = 0;         // quando muda para 1, não reescreve o arquivo no cartão SD
RTC_DATA_ATTR unsigned long contador_1 = -1;  // a contagem está na primeira linha do setup().
// -------------------------------------------------------------------
void setup() {
  // Setup aqui
  // -------------------------------------------------------------------
  contador_1 += 1;     // O contador serve para contar até 23 e enviar os dados para o servidor (24 = 4 minutos)
  Serial.begin(9600);  // velocidade de transmissão para serial, taxa de 9600 baud
  // -------------------------------------------------------------------
  if (boot == 0) {
    // Cartão SD e arquivo:
    SD.begin();
    writeFile(SD, "/Teste_1.txt", "\n");
    escreve_string("n° escrita, bmp_temp, bmp_pres, bmp_alt, TT101, TT102, TT103, TT104, TT105, TT106, nivelBateria, Gx, Gy, Gz, Ax, Ay, Az, mpu_temp, LAT_GPS, LGN_GPS, KMPH_GPS, SAT_GPS, ALTURA_GPS, gps_pres");
    boot = 1;
  }
  // -----------------------------------------------------------------
  // Multiplexador:
  pinMode(S0, OUTPUT);  // define Sn, como saída, onde n vai de 0-3, como consta na inicialização
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  setMux(0);
  // -------------------------------------------------------------------
  // SENSOR MPU6050:
  mpu.begin();                                   // inicializa o girosccópio/acelerômettro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // define faixa de medição do acelerômetro
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // define faixa de medição do giroscópio
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);     // define precisão da leitura
  //------------------------------------------------------------------
  // SENSOR BMP280:
  bmp.begin(0x76);  // inicializa o sensor bmp280
  // Dados padrões de configuração do datasheet:
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Modo de operação
                  Adafruit_BMP280::SAMPLING_X2,      // Sobreamostragem de temperatura
                  Adafruit_BMP280::SAMPLING_X16,     // Sobreamostragem de pressão
                  Adafruit_BMP280::FILTER_X16,       // Filtragem
                  Adafruit_BMP280::STANDBY_MS_500);  // Período de descanço (standby)
  // -------------------------------------------------------------------
  // loop do GPS: aqui é feita a requisição de dados
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);  // inicializa o módulo, atribuindo baud-rate e serial
  for (unsigned long start = millis(); millis() - start < intervalo_gps;) {
    while (neogps.available()) {
      gps.encode(neogps.read());  // faz a requisição durante 1000 ms, enviando vários pedidos
    }
  }
  // --- ETAPA DE LEITURA ---
  dados_gps();               // Dados do GPS: ALTURA_GPS, LAT_GPS, LGN_GPS, KMPH_GPS, SAT_GPS
  pressao_calc(ALTURA_GPS);  // Calcula pressão com "ALTURA_GPS": gps_pres
  leitura_bmp();             // Dados do bmp: bmp_pres, bmp_alt, bmp_temp
  leituraMux();              // Termistor e nível de bateria: valorT[j], nivelBateria
  mpu_6050();                // Giroscópio: Gx, Gy, Gz, Ax, Ay, Az, mpu_gyro, mpu_acel
  // --- ETAPA DE ESCRITA ---
  SD.begin();                                     // Inicializa o cartão SD para escrita
  appendFile(SD, "/Teste_1.txt", "\nEscrita: ");  // Nomeia escrita de cada dado, usando a contagem como referência
  contador_0 += 1;                                // Enumera (itera) a contagem de escritas no cartão SD
  escreve_string(String(contador_0));
  // Dados do BMP280:
  escreve_float(bmp_temp);  // escreve dado em float para string (olhar função escreve_float()), temperatura bmp280
  escreve_float(bmp_pres);  // pressão bmp280
  escreve_float(bmp_alt);   // altura bmp280
  // Dados do MUX:
  escreve_float(valorT[0]);   // temperatura termistor TT101
  escreve_float(valorT[1]);   // temperatura termistor TT102
  escreve_float(valorT[2]);   // temperatura termistor TT103
  escreve_float(valorT[3]);   // temperatura termistor TT104
  escreve_float(valorT[4]);   // temperatura termistor TT105
  escreve_float(valorT[5]);   // temperatura termistor TT106
  escreve_int(nivelBateria);  // escreve dado em int para string (olhar função escreve_int()), percentual da bateria
  // Dados do giroscópio/acelerômetro:
  escreve_float(Gx);        // eixo X giroscópio em °/s
  escreve_float(Gy);        // eixo Y giroscópio em °/s
  escreve_float(Gz);        // eixo Z giroscópio em °/s
  escreve_float(Ax);        // eixo X acelerômetro em m/s²
  escreve_float(Ay);        // eixo Y acelerômetro em m/s²
  escreve_float(Az);        // eixo Z acelerômetro em m/s²
  escreve_float(mpu_temp);  // temperatura do MPU6050 em °C
  // Dados do GPS:
  escreve_float(LAT_GPS);     // latitude do GPS
  escreve_float(LGN_GPS);     // longitude do GPS
  escreve_float(KMPH_GPS);    // velocidade do GPS em km/h
  escreve_int(SAT_GPS);       // satélites conectados
  escreve_float(ALTURA_GPS);  // altura do GPS em metros (m)
  escreve_float(gps_pres);    // pressão calculada com altura do GPS em Pascal (Pa)
  // --- ETAPA DE ESCRITA NA SERIAL ---
  /*
  Serial.println("Temperatura (bmp280): " + String(bmp_temp, 2));
  Serial.println("Pressão (bmp280): " + String(bmp_pres, 2));
  Serial.println("Altitude (bmp280): " + String(bmp_alt, 2));
  Serial.println("TT101: " + String(valorT[0], 2));
  Serial.println("TT102: " + String(valorT[1], 2));
  Serial.println("TT103: " + String(valorT[2], 2));
  Serial.println("TT104: " + String(valorT[3], 2));
  Serial.println("TT105: " + String(valorT[4], 2));
  Serial.println("TT106: " + String(valorT[5], 2));
  Serial.println("Bateria: " + String(nivelBateria));
  Serial.println("Giroscópio (mpu6050): " + mpu_gyro);
  Serial.println("Acelerômetro (mpu6050): " + mpu_acel);
  Serial.println("Temperatura (mpu6050): " + String(mpu_temp, 2));
  Serial.println("Latitude (gps): " + String(LAT_GPS, 2));
  Serial.println("Longitude (gps): " + String(LGN_GPS, 2));
  Serial.println("Velocidade (gps): " + String(KMPH_GPS, 2));
  Serial.println("Satélites conectados (gps): " + String(SAT_GPS));
  Serial.println("Altitude (gps): " + String(ALTURA_GPS));
  Serial.println("Pressão (gps): " + String(gps_pres, 2));
  Serial.println("Boot: " + String(boot));
  */
  t0 = millis();  // grava tempo total desde que ligou o ESP32, leu dados + escrita
  /*
  if (contador_1 == 23) {                                  // se chegou em 230000 milisegundos, inicializa wi-fi
    t1 = millis();                                         // grava tempo antes de iniciar wi-fi
    WiFi.begin(ssid, password);                            // inicializa wi-fi com nome e senha da rede local
    while (WiFi.status() != WL_CONNECTED && t6 <= 9800) {  // se o status de conexão do wi-fi é de "conectado" e tempo passado <= 9800ms, executa o comando
      delay(100);                                          // gera atrasos de 100ms até se conectar ao wi-fi
      t6 += 100;                                           // memoriza tempo descorrido
      if (t6 == 9900)                                      // segunda verificação para o caso de NÃO conexão com wi-fi. Evita que o código trave no loop atual
      {
        DeepSleep(9998);                                   // inicia o Deep-Sleep com 9998ms, leva 1ms para pular para a função deep-sleep, sobrando 1ms para evitar o 0ms
      }
    }
    t2 = millis() - t1;                                           // grava tempo após conexão com wi-fi e subtrai do tempo antes de conectar (t1)
    if (WiFi.status() == WL_CONNECTED && ALTURA_GPS < 8900.00) {  // se o wi-fi está conectado e a altura lida no GPS é inferior a 8900.00 m
      t3 = 9800 - (t2 + t5);                                      // calcula o tempo do ciclo desconsiderando t2 e t5, onde t5 é o tempo de envio dos dados (nenhum envio = 0)
      if (t3 < 0)                                                 // para casos onde (t2+t5) > 9800, o valor de t3 será negativo, travando o microcontrolador
      {
        t3 = 1;  // define valor 1ms (evitar zero)
      }
      delay(t3);                                                         // aguarda o tempo descorrido que foi calculado anteriormente
      t4 = millis();                                                     // grava o tempo de início do envio dos dados
      conexao(0);                                                        // é feita a conexão com as variáveis globais dos dados lidos nos sensores, esse processo dura 1,1-1,2 segundos
      WiFi.disconnect();                                                 // desconecta o wi-fi após envio dos dados
      contador_1 = 0;                                                    // zera o contador da transmissão para um novo ciclo
      t5 = millis() - t4;                                                // grava o tempo de final de envio dos dados, desconexão wi-fi e reset do contador para ser usado na próxima conexão
    } else if (WiFi.status() == WL_CONNECTED && ALTURA_GPS > 8900.00) {  // se o wi-fi está conectado e a altura for superior a 8900.00 m
      t3 = 9800 - (t2 + t5);
      if (t3 < 0) {
        t3 = 1;
      }
      delay(t3);
      t4 = millis();
      conexao(1);
      WiFi.disconnect();
      contador_1 = 0;
      t5 = millis() - t4;
    } else {
      // Serial.println("WiFi Desconectado");
      DeepSleep(10000);  // com 10000ms x = 10000000 (ver função DeepSleep()), logo o tempo de hibernar é igual a 0s
    }
  
  }
  */
  DeepSleep(t0);  // inicia modo Deep-Sleep com tempo de escrita_final descontando de 10 segundos
}
// ------------------------------------------------------------------------
void writeFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Escrito para o arquivo: %s\n", path);
  File file = fs.open(path, FILE_WRITE);  // abre o arquivo para escrita, path = arquivo; FILE_WRITE = conteúdo a ser escrito

  if (!file) {
    //Serial.println("Falhou em abrir arquivo para escrita");
    //return;
  }
  if (file.print(message)) {
    //Serial.println("Arquivo escrito");
  } else {
    //Serial.println("Falhou em escrever");
  }

  file.close();  // fecha arquivo após escrita
}
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Atribuido para o arquivo: %s\n", path);
  File file = fs.open(path, FILE_APPEND);  // adiciona conteúdo ao arquivo criado em writeFile, path = arquivo; FILE_APPEND = conteúdo a ser atribuído;

  if (!file) {
    //Serial.println("Falha em abrir arquivo para atribuição");
    //return;
  }
  if (file.print(message)) {
    //Serial.println("Mensagem atribuída");
  } else {
    //Serial.println("Falha na atribuição");
  }

  file.close();
}
// ----------------------------FUNÇÕES GERAIS--------------------------------
void mpu_6050() {
  sensors_event_t a, g, temp;   // quando ocorre variação em a, aceleração, g, giro e t, temperratura
  mpu.getEvent(&a, &g, &temp);  // atribui os valores de aceleração, giro ee temperatura em a, g e temp
  // Leitura acelerômetro (m/s²):
  Ax = a.acceleration.x;
  Ay = a.acceleration.y;
  Az = a.acceleration.z;
  // Leigura giroscópio (°/s):
  Gx = g.gyro.x;
  Gy = g.gyro.y;
  Gz = g.gyro.z;
  mpu_temp = temp.temperature;
  mpu_gyro = "[" + String(Gx, 2) + "," + String(Gy, 2) + "," + String(Gz, 2) + "]";  // unidade em LSB/(°/s)
  mpu_acel = "[" + String(Ax, 2) + "," + String(Ay, 2) + "," + String(Az, 2) + "]";  // unidade em LSB/(g/s)
}
void pressao_calc(float h_)  // recebe altura do GPS
{
  double p = (101325) * pow((1 - (2.25577E-5) * h_), 5.25588);
  gps_pres = (float)p;  // converte de double para float
  // Fonte da equação: https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
}
void dados_gps() {
  if (gps.location.isValid() == 1) {     // se há dados coletados, atribui nas variáveis globais
    LAT_GPS = gps.location.lat();        // latitude
    LGN_GPS = gps.location.lng();        // longitude
    KMPH_GPS = gps.speed.kmph();         // velocidade
    SAT_GPS = gps.satellites.value();    // satélites conectados
    ALTURA_GPS = gps.altitude.meters();  // altura
  } else {                               // se não há dados, mantém valores zerados como float
    LAT_GPS = 0.0;
    LGN_GPS = 0.0;
    KMPH_GPS = 0.0;
    SAT_GPS = 0;
    ALTURA_GPS = 0.0;
  }
}
String payload() {
  String dados = "[" + String(LAT_GPS, 2) + "," + String(LGN_GPS, 2) + "," + String(KMPH_GPS, 2) + "," + String(SAT_GPS) + "," + String(ALTURA_GPS, 2) + "," + String(gps_pres, 2) + "," + String(nivelBateria) + "," + String(bmp_alt, 2) + "," + String(bmp_temp, 2) + "," + String(mpu_temp, 2) + "," + String(valorT[0], 2) + "," + String(valorT[1], 2) + "," + String(valorT[2], 2) + "," + String(valorT[3], 2) + "," + String(valorT[4], 2) + "," + String(valorT[5], 2) + "]";
  return dados;
  // A formatação segue a do servidor, formato JSON
  // Todos os valores são convertidos para String, em especial float recebe formatação com duas casas decimais, ex: String(numero, 2)
}

void leitura_bmp() {
  bmp_pres = bmp.readPressure();        // faz leitura de pressão
  bmp_temp = bmp.readTemperature();     // faz leitura de temperatura
  bmp_alt = bmp.readAltitude(1013.25);  // faz leitura de altitude usando como referência 1013.25 m (nível do mar)
}
void leituraMux() {
  float valorM = 0;                                                                              // média dos valorres inicializa em 0
  for (int j = 0; j < canais; j++) {                                                             // laço de tamanho 0-7
    setMux(pinos_canais[j]);                                                                     // atribui cada valor em pinos_canais[]
    for (int i = 0; i < 5; i++) {                                                                // itera 5 vezes
      valorM += (float)0.9792 * (floatMap(analogRead(analog), 0.0, 4095.0, 0.0, 3.3)) + 0.4302;  // para cada iteração é feita leitura analógica
      delay(50);                                                                                 // atraso para o ADC ler com sucesso
    }                                                                                            // reinicia o laço para cada pino 5 vezes
    valorM = (float)valorM / 5.0;                                                                // faz a média das 5 leituras
    valor[j] = valorM;                                                                           // armazena o valor da iteração atual em valor[j]
    valorM = 0;                                                                                  // zera valorM para a próxima iteração de j
  }
  for (int i = 0; i < canais - 1; i++) {  //converte valores de tensão em Temperatura
    valorT[i] = A[i] * (pow(valor[i], 4.0)) + B[i] * (pow(valor[i], 3.0)) + C[i] * (pow(valor[i], 2.0)) + D[i] * valor[i] + E[i];
    // Atribui curva de calibração para cada termistor, dado uma leitura analógica
  }
  float tensao_b = valor[6] * 3.0;                                //leitura analógica da bateria, divisor de tensão
  nivelBateria = floatMap(tensao_b, 3 * 2, 4.2 * 2, 0.0, 100.0);  //converte em porcentagem o valor lido na bateria
  if (nivelBateria < 0) {                                         // ajuste para evitar erro de syntaxe
    //Serial.print("Erro de leitura na bateria (valor < 0): "); Serial.println(nivelBateria);
    nivelBateria = 0;
  } else if (nivelBateria > 100) {  // ajuste para evitar erro de syntaxe
    //Serial.print("Erro de leitura na bateria (valor > 0): "); Serial.println(nivelBateria);
    nivelBateria = 0;
  }
}
void setMux(short inputCH) {
  switch (inputCH) {
    case 0:  // combinação binária para leitura dos pinos de 0-6
      digitalWrite(S0, LOW);
      digitalWrite(S1, LOW);
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case 1:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case 2:
      digitalWrite(S0, LOW);
      digitalWrite(S1, HIGH);
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case 3:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, HIGH);
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case 4:
      digitalWrite(S0, LOW);
      digitalWrite(S1, LOW);
      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      break;
    case 5:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      break;
    case 6:
      digitalWrite(S0, LOW);
      digitalWrite(S1, HIGH);
      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      break;
    case 15:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, HIGH);
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      break;
  }
}
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
String dados(int _e, int _b, float _t, float _p, String G, String A, String PL) {
  // equipe, lv bateria, temperatura, pressão, giroscópio, acelerômetro, payload (termistor).
  String dados = " {\"equipe\":" + String(_e) + ",\"bateria\":" + String(_b) + ",\"temperatura\":" + String(_t, 2) + ",\"pressao\":" + String(_p, 2) + ",\"giroscopio\":" + G + ",\"acelerometro\":" + A + ",\"payload\":" + PL + "}";
  return dados;
  // São os dados enviados, também concatena o payload
}
/*
void conexao(int estado) {                             // estado 0 = bmp, estado 1 = gps e mpu_temp
  String toSend;                                       // cria variável string que armazena mensagem a ser enviada
  HTTPClient http;                                     // renomeia biblioteca HTTPClient para http
  http.begin(serverName.c_str());                      // inicializa comunicação http com endereço http definido
  http.addHeader("Content-Type", "application/json");  // define o formato da mensagem como JSON
  if (estado == 1) {
    toSend = dados(50, nivelBateria, mpu_temp, gps_pres, mpu_gyro, mpu_acel, payload());  // temperatura do mpu6050 e pressão calculada com altura do gps
    if (Serial.available()) {
      Serial.println(toSend);
    }
  } else if (estado == 0) {
    toSend = dados(50, nivelBateria, bmp_temp, bmp_pres, mpu_gyro, mpu_acel, payload());  // temperatura do bmp280 com pressão própria do bmp280
    if (Serial.available()) {
      Serial.println(toSend);
    }
  }
  int httpResponse = http.POST(toSend.c_str());  // tipo de envio, POST, cujo conteúdo é a variável toSend de string para char
  http.end();                                    // encerra comunicação http
}
*/
void escreve_string(String _conteudo) {
  char_conteudo = _conteudo.c_str();              // recebe conteúdo em string e converte para char
  appendFile(SD, "/Teste_1.txt", char_conteudo);  // atribui no arquivo o conteúdo em char
  appendFile(SD, "/Teste_1.txt", ",");            // virgula separador de dado
}
void escreve_int(int _conteudo) {
  s_conteudo = String(_conteudo);                 // recebe conteúdo em inteiro e converte para string
  char_conteudo = s_conteudo.c_str();             // recebe conteúdo em string e converte para char
  appendFile(SD, "/Teste_1.txt", char_conteudo);  // atribui no arquivo o conteúdo em char
  appendFile(SD, "/Teste_1.txt", ",");
}
void escreve_float(float _conteudo) {
  s_conteudo = String(_conteudo, 2);              // recebe conteúdo em float e converte para float com 2 casas decimais
  char_conteudo = s_conteudo.c_str();             // recebe conteúdo em string e converte para char
  appendFile(SD, "/Teste_1.txt", char_conteudo);  // atribui no arquivo o conteúdo em char
  appendFile(SD, "/Teste_1.txt", ",");
}
// ------------------------------------------------------------------------
void DeepSleep(unsigned long x) {
  // DEEP SLEEP:
  esp_sleep_enable_timer_wakeup(10000000 - (x * 1000));
  // para um dado x (no intervalo de 0-10000ms) multiplica por 1000ms, converte para microsegundo e subtrai dos 10 segundos (10000000us)
  esp_deep_sleep_start();
  // inicia o modo deep-sleep
}
void loop() {
}