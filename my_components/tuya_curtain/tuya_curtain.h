#pragma once

#ifndef TUYA_COVER_H
#define TUYA_COVER_H

#include <Arduino.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include <ctime>
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/text_sensor/text_sensor.h"

//#define TCURT_PRINT_RAW_PROTO    // отладочный вывод протокола в лог
//#define TCURT_NDMODE             // активация режима день/ночь
//#define TCURT_AUTOPOWER          // активация режима Auto Power
//#define TCURT_COUNTDOWN          // активация режима CountDown
//#define TCURT_MOTOR_REVERS 1     // настройка реверса мотора, 1 - значение
//#define TCURT_RESTORE 10000      // восстановление настроек после перезагрузки, шторам нужно открыться и закрыться, 10000 - период сохранения данных в ms
//#define TCURT_VIRTUAL_POS 500    // режим рассчета позиции мотора во время работы, 500 - период обновления в ms
//#define TCURT_CALIBRATE_TIMEOT 20000 // контроль завсания во время калибровки
//#define TCURT_SILENT_CALIBRATE      // флаг тихой калибровки

// типы пакетов
enum tCommand:uint8_t {HEARTBEAT=0,            //- периодический HEARTBEAT 
                       PRODUCT_QUERY=1,        //- идентификатор устройства
                       CONF_QUERY=2,           //- статус наличия ног управления диодом WIFI и ноги с кнопкой 
                       WIFI_STATE=3,           //- статус связи
                       WIFI_RESET=4,           //- ресет
                       WIFI_SELECT=5,          //- pairing mode
                       DATAPOINT_DELIVER = 6,  //- установка данных в MCU
                       DATAPOINT_REPORT = 7,   //- ответ на запрос данных
                       DATAPOINT_QUERY=8,      //- запрос настроек
                       LOCAL_TIME_QUERY=0x1C,  //- синхронизация времени
                       GET_NETWORK_STATUS=0x2B //- подтверждение получения ответа на запрос о статусе сети
};

// типы инфы о состоянии связи
enum tMainState:uint8_t {wsPair=0,      // SmartConfig pairing status
                         wsNoConf=1,    // AP pairing status - устройство не имеет настроек WIFI
                         wsWifiConf=2,  // Wi-Fi has been configured, but not connected to the router - устройство имеет настройки wifi, но не подключено
                         wsWifi=3,      // Wi-Fi has been configured, and connected to the router - успешно подключено и нужен первый набор данных
                         wsWifiCloud=4, // Wi-Fi has been connected to the router and the cloud - нужен второй набор данных
                         wsLowPow=5,    // Wi-Fi device is in the low power consumption mode
                         wsAPSmart=6    // Both SmartConfig and AP pairing status are available
};

// булевы переменные протокола
enum tStat:uint8_t {OFF=0, ON=1, UNDEF=0xFF};

// типы переменных 
enum tDataType:uint8_t {dtBool=1, dtVal=2, dtEnum=4, dtBits=5};

// значение данных
enum tDpid:uint8_t {idControl=1,
                    idQue_Percent=2,
                    idRep_Percent=3,
                    idMode=4,
                    idDir=5,
                    idPower=6,
                    idState=7,
                    idCountDown=8, // не работает
                    idLeftTime=9,
                    idTotalTime=10,
                    idSituation=11,
                    idFault=12
};

// состояния режима Control
enum tEnControl:uint8_t {tenOpen=0, tenStop=1, tenClose=2};

// структура хранения данных во флеше
struct sCurtianSave{
   uint8_t pos=0xFF;
   float speed=0;
};

// Tested device version {"p":"wraaecjqfiirj8go","v":"1.0.0","m":0} 

namespace esphome {
namespace tuya_curtain {

using namespace esphome;
using namespace esphome::text_sensor;
using namespace esphome::cover;

static const char *const TAG = "TuyaCurtain";

using cover::CoverCall;
using cover::CoverOperation;
using text_sensor::TextSensor;    
using uart::UARTDevice;  
using uart::UARTComponent;

// настройки таймаутов
constexpr uint32_t HEARTBEAT_INTERVAL = 15; // переодичность передачи сигнала активности процессору 
constexpr uint32_t UART_TIMEOUT = 300; //время ожидания uart (mS), пока это время не истечет после приема или отправки - ждем
constexpr uint32_t SEND_TIMEOUT = 300; //время ожидания между сеансами отправки данных (mS)

class TuyaCurtain : public cover::Cover, public Component {

 private:
    const uint8_t COMMAND_START[2] = {0x55, 0xAA}; // хидер протокола обмена
    esphome::text_sensor::TextSensor *sensor_mcu_id_{nullptr}; // MCU продукт ID
    // указатель на UART, по которому общаемся с кондиционером
    esphome::uart::UARTComponent *_tuya_serial;
    uint8_t receivedCommand[254]; // буфер чтения
    uint8_t commandLength=0; // длинна команды определяется во время получения пакета
    int16_t receiveIndex=0; // количество принятых байт
    uint8_t sendCounter=0; //маршрутизатор отправки
    uint8_t waitReply=0xFF; //буфер крайней отправленной команды, для контроля ответа
    uint32_t lastSend=0; // время отправки последнего байта
    uint32_t lastRead=0; // время получения последнего байта
    bool sendRight=false; //флаг разрешения внеочередной отправки данных, когда идентификатор полученного ответа от терстата совпал с запросом
    uint32_t heatBearTimer=0; // таймер контроля получения ответа на пинг
    uint8_t proto_vers=0; // версия MCU , от этого зависит протокол
    // для передачи статуса связи
    uint8_t oldNetState=0xFF;
    uint8_t netState=0xFF;
    // нога статуса WIFI
    GPIOPin* status_pin_{nullptr};
    int16_t status_pin_reported_=-1;
    // нога контроля сигнала ресет протокол от MCU
    GPIOPin* reset_pin_{nullptr};
    int16_t reset_pin_reported_=-1;
    // флаги контроля выполнения  
    bool _set_stop=false; // флаг команды останнова
    bool _set_open=false;
    bool _set_close=false;
    uint8_t _set_pos=0xFF; // запрошенная позиция в процентах !!!
    bool _no_calibarte=true; // флаг ошибки калибровки мотора
    #ifdef TCURT_VIRTUAL_POS
       uint8_t _old_pos=0xFF; // буфер старого положения штор в процентах !!!
       uint8_t _dest_pos=0xFF; // цель позционирования в процентах !!!
       float _speed=0; // скорость позиционирования штор, рассчитывается
       uint32_t _timer_pos=0; // таймер старта позиционирования штор
       uint32_t _timer_update=0; // таймер визуализации позиционирования
    #endif
    #ifdef TCURT_NDMODE   
       uint8_t _set_mode=0xFF; // установка день/ночь
    #endif
    #ifdef TCURT_AUTOPOWER   
       uint8_t _set_autopower=0xFF; // какой то автоповер
    #endif
    #ifdef TCURT_COUNTDOWN   
       uint8_t _set_countdownn=0xFF;
    #endif
    #ifdef TCURT_RESTORE // режим восстановления после отключения питания
       sCurtianSave saveData; 
       uint32_t saveTimer=0; 
       uint32_t waiteTimer=0;
       uint8_t calibrate_count=0;
       uint8_t restoreCounter=0; // маршрутизатор процедуры восстановления       
       ESPPreferenceObject storage = global_preferences->make_preference<sCurtianSave>(this->get_object_id_hash());
       // проверка, при необходимости сохранение данных
       void saveDataFlash(uint8_t pos=0xFF){
          if(restoreCounter==6){// имеет смысл только при калиброванном моторе
             return;
          }
          bool needSave=false;
          #ifdef TCURT_VIRTUAL_POS
             if(_speed!=0 && saveData.speed!=_speed){
                saveData.speed=_speed;
                needSave=true;
             }
          #endif    
          if(pos==0xFF){
             pos=this->position*100;
          }
          if(saveData.pos!=pos){
             saveData.pos=pos;
             needSave=true;
          }
          if(needSave){
             if (storage.save(&saveData) && global_preferences->sync()){ // данные успешно сохранены
                #ifdef TCURT_VIRTUAL_POS
                   ESP_LOGV(TAG, "Data store to flash, speed:%f, pos:%d", saveData.speed, saveData.pos); 
                #else
                   ESP_LOGV(TAG, "Data store to flash, pos:%d", saveData.pos); 
                #endif
             } else {
                ESP_LOGE(TAG, "Data store to flash - ERROR !");
             }
          }
       }

       // получение данных восстановления из флеша
       inline bool loadDataFlash(){
          if(storage.load(&saveData)){ // читаем сохраненные данные
             #ifdef TCURT_VIRTUAL_POS
                if(saveData.speed!=0){
                   _speed=saveData.speed;   
                }
                ESP_LOGV(TAG, "Stored data loaded, speed:%f, pos:%d", saveData.speed, saveData.pos); 
             #else
                ESP_LOGV(TAG, "Stored data loaded, pos:%d", saveData.pos); 
             #endif
             return true;
          } else {
             ESP_LOGE(TAG, "Stored data load - ERROR !"); 
          }
          return false;
       }
    #endif

    // считать значение из двух байт INDIAN
    inline uint16_t get16(uint8_t* data){
       uint8_t buff[2]={data[1],data[0]};
       return *((uint16_t*)(&buff));
    }
    
    // считать значение из 4 байт INDIAN
    uint32_t get32(uint8_t* data){
       uint8_t buff[4]={data[3],data[2],data[1],data[0]};
       return *((uint32_t*) (&buff));
    }

    // загрузить значение в буфер в формате indian
    void put32(uint8_t* buff, uint32_t val){
       buff[3]=(uint8_t)val;
       val/=0x100;
       buff[2]=(uint8_t)val;
       val/=0x100;
       buff[1]=(uint8_t)val;
       val/=0x100;
       buff[0]=(uint8_t)val;
    }

    // получить текст из буфера данных
    String getStringFromBuff(uint8_t* buff, uint8_t count){
       String text="";
       for(uint8_t i=0; i<count; i++){
          text+=(char)buff[i];
       }
       return text;
    }

    void _debugMsg(const String &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ...) {
        va_list vl;
        va_start(vl, line);
        esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
        va_end(vl);
    }

    // для тестовой печати протокола обмена
    void _debugPrintPacket(uint8_t *packet, uint8_t length, bool dirrect, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0) {
        String st = "";
        char textBuf[11];
        // заполняем время получения пакета
        memset(textBuf, 0, 11);
        sprintf(textBuf, "%010u", esphome::millis());
        st = st + textBuf + ": ";
        // формируем преамбулы
        if (dirrect) {
            st += "Out: ";  // преамбула входящего пакета
        } else  {
            st += " In: ";  // преамбула исходящего пакета
        }
        // формируем данные
        for (size_t i = 0; i < length; i++) {
            // для нормальных пакетов надо заключить заголовок в []
            if (i == 0) st += "[";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "[";

            memset(textBuf, 0, 11);
            sprintf(textBuf, "%02X", packet[i]);
            st += textBuf;

            // для нормальных пакетов надо заключить заголовок в []
            if (i==5) st += "]";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "]";

            st += " ";
        }
        if (line == 0) line = __LINE__;
        _debugMsg(st, dbgLevel, line);
    }

    // очистка буфер приема
    void resetAll() {
       receiveIndex = -1;
       commandLength = -1;
    }
/////////////////////// ОТПРАВКА ИСХОДЯЩИХ ПАКЕТОВ ///////////////////////

// компоновка и отправка пакета MCU
    void sendCommand(uint8_t comm, uint8_t length=0, uint8_t* data=nullptr){
       if (_tuya_serial!=nullptr){
          sendRight=false; // снять флаг ускорения передачи данных
          uint8_t controll_buffer[length+1+6]={COMMAND_START[0],COMMAND_START[1],0,0,0,0};           
          uint8_t chkSum=COMMAND_START[0]+COMMAND_START[1]; //контрольная сумма
          controll_buffer[3]=comm; // команда
          controll_buffer[5]=length; //размер буфера данных
          _tuya_serial->write_array(controll_buffer,6); // отправляем хидер
          chkSum+=comm;
          chkSum+=length;
          _tuya_serial->write_array(data,length); // отправляем тело
          for(uint8_t i=0; i<length; i++){ // считаем КС
             chkSum+=data[i];
          }
          memcpy(controll_buffer+6, data, length); // сохраняем данные для лога
          _tuya_serial->write_array(&chkSum,1); // отправляем КС
          controll_buffer[length+6]=chkSum;
          #ifdef TCURT_PRINT_RAW_PROTO
             _debugPrintPacket(controll_buffer, length+6+1, true, TCURT_PRINT_RAW_PROTO);          
          #endif
          lastSend=esphome::millis(); //время отправки последнего байта
          waitReply=comm; // запоминаем команду которую отправили
       } else {
          uint32_t timer=-10000;
          if(esphome::millis()-timer>10000){
             timer=esphome::millis();
             ESP_LOGE(TAG,"There is no UART port, work is impossible");
          }
       }
    }

//отправка тестовых запросов
    void sendTest(uint8_t comm){
       ESP_LOGE("!!!!!!!!!","Send TEST command: %u", comm);
       sendCommand((uint8_t)comm);  
    }
    
//отправка статусных запросов
    void sendComm(tCommand comm){
       ESP_LOGV(TAG,"Send short command: %u", comm);
       sendCommand((uint8_t)comm);  
    }

//отправка  статусных команд
    void sendComm(tDpid comm, tDataType dataType, uint8_t data){
       ESP_LOGV(TAG,"Send command Id:%02X, valType:%d, data:%d", comm, dataType, data);
       uint8_t buff[8]={comm, dataType,0,0,0,0,0,0};
       if(dataType==dtVal){ // отправка значения 4 байта
          buff[3]=4;  
          put32(buff+4, data);   
       } else { // отправка значения 1 байт
          buff[3]=1;
          buff[4]=data;
       }
       sendCommand(DATAPOINT_DELIVER, 4+buff[3], buff);   
    }

//получение текущего статуса связи
    uint8_t getNetState() {
       uint8_t status = wsWifiConf;
       if(network::is_connected()) {
          // если контролировать подключение к серверу ХА, то это повлечет перегрузку, а это не нужно
          if(proto_vers>=3/* && (api_is_connected() || mqtt_is_connected())*/){
             status=wsWifiCloud;
          } else {
             status=wsWifi;
          }
       } else {
          #ifdef USE_CAPTIVE_PORTAL
             if (captive_portal::global_captive_portal != nullptr && captive_portal::global_captive_portal->is_active()) {
                status = wsNoConf;
             }
          #endif
       }
       return status;
    }

//отправка инфо о статусе модуля связи, это влияет на отдачу данных в модуль связи
    void sendNetState(uint8_t status=wsWifiConf) {
       ESP_LOGD(TAG,"Send main net status: %u", status);
       sendCommand(WIFI_STATE, 1, &status);
    }

// отправка статуса связи
    void sendNetworkStatus(){
       uint8_t payload=getNetState();
       ESP_LOGD(TAG,"Send Network Status: %d", payload);
       sendCommand(GET_NETWORK_STATUS,1,&payload);   
    }

    
// Разбор служебных пакетов
    bool processCommand(uint8_t commandByte/*3*/, uint16_t length/*5*/) {
       bool knownCommand = false;
       switch (commandByte) {
         case HEARTBEAT: {
           if(receivedCommand[6]==OFF){
              ESP_LOGD(TAG,"Get first reply HEARTBEAT");
              heatBearTimer = esphome::millis();
              knownCommand = true;
           } else if(receivedCommand[6]==ON){
              ESP_LOGD(TAG,"Get every reply HEARTBEAT after %d sec",esphome::millis()/1000);
              heatBearTimer = esphome::millis();
              knownCommand = true;
           } else {
              ESP_LOGE(TAG,"Get reply HEARTBEAT: %d - ERROR !!!", receivedCommand[6]);
              break;
           } 
           if(sendCounter==1){
               sendCounter=2;
           }
           break;
         }
         case PRODUCT_QUERY: { // идентификатор изделия Query product information
           String id=getStringFromBuff((uint8_t*)receivedCommand+6,length);
           ESP_LOGD(TAG,"Get Product info: %s", id.c_str());
           if(sensor_mcu_id_!=nullptr){
              sensor_mcu_id_->publish_state(id.c_str());   
           }
           knownCommand = true;
           if(sendCounter==2){ // пришел ответ на команду, переходим дальше
              sendCounter=3;
           }
           break;
         }
         case CONF_QUERY: { //0x55 0xAA  0x01(03)  0x02  0x00  0x00  CS  Query working mode of the module set by MCU
           ESP_LOGD(TAG,"Get Stat Confirm reply");
           if(length==0){ // нет дополнительных ног
              knownCommand = true;
           } else if(length==4){ // есть дополнительные ноги
              status_pin_reported_=get16(receivedCommand+6);
              reset_pin_reported_=get16(receivedCommand+8);
              char one[]={(char)('A'+receivedCommand[6]),0};
              char two[]={(char)('A'+receivedCommand[8]),0};
              ESP_LOGI(TAG,"Get pins setting: WiFi pin is P%s_%u, Reset pin is P%s_%u.",one,receivedCommand[7],two,receivedCommand[9]);
              if(this->status_pin_== nullptr){
                 ESP_LOGE(TAG,"It is necessary to configure status_pin, perhaps it is P%s_%u.",one,receivedCommand[7]);
              } else {
                 if (!((this->status_pin_)->is_internal()) || ((InternalGPIOPin *)(this->status_pin_))->get_pin() != receivedCommand[7]){
                    ESP_LOGW(TAG,"The configured status_pin does not meet TuyaMcu requirements, must be P%s_%u. Errors are possible.",one,receivedCommand[7]);
                 }
              }
              if(this->reset_pin_== nullptr){
                 ESP_LOGE(TAG,"It is necessary to configure reset_pin, perhaps it is P%s_%u.",two,receivedCommand[9]);
              } else {
                 if (!((this->reset_pin_)->is_internal()) || ((InternalGPIOPin *)(this->reset_pin_))->get_pin() != receivedCommand[9]){
                    ESP_LOGW(TAG,"The configured reset_pin does not meet TuyaMcu requirements, must be P%s_%u. Errors are possible.",two,receivedCommand[9]);
                 }      
              }
           }
           if(sendCounter==3){
             sendCounter=4; // переход к передаче статуса wifi
           }
           break;
         }
         case WIFI_STATE: { //55 aa 01(03) 03 00 00 CS,  Report the network status of the device
           ESP_LOGD(TAG,"Get Confirm Network reply");
           knownCommand = true; // пришло подтверждение получения сетевого статуса
           oldNetState=netState; // снимаем признак отправки статуса
           if(sendCounter==4){
              sendCounter=5;
           }
           break;
         }
         case WIFI_RESET: { //55 aa 01(03) 04 00 00 CS Reset Wi-Fi
           ESP_LOGD(TAG,"Get Reset from MCU");
           sendComm(WIFI_RESET); // отвечаем что приняли ресет
           sendCounter=1; // запускаем карусель
           knownCommand = true;
           break;
         }
         case WIFI_SELECT: {  //55 aa 01(03) 05 00 00 Reset Wi-Fi and select the pairing mode  //ресет - мигает диод
           ESP_LOGD(TAG,"Get Pairing mode");
           knownCommand = true;
           break;
         }
         case LOCAL_TIME_QUERY: {  //55 aa 01(03) 1c 00 00 1c запрос времени (в протоколе ТУИ это другая команда (WTF) ???)
           ESP_LOGE(TAG,"Get Date/time request - unsupported");
           knownCommand = false;
           break;
         }
         case GET_NETWORK_STATUS: {  //0x2B 55;AA;03;2B;00;00;2D Get the MAC address of the module
           ESP_LOGD(TAG,"Get Network status request");
           knownCommand = true;
           sendNetworkStatus();
           break;
         }
         default:
           ESP_LOGE(TAG,"Get Unexpect request 0x%X",commandByte);
           break;
       }
       return knownCommand;
    }

// Разбор данных устройства
    bool processStatusCommand(uint8_t dpid, uint32_t data) {
       bool ret=false;
       auto temp_operation=this->current_operation;
       auto temp_position=this->position;
       switch (dpid) {
         case idControl: { //open, stop, close
            if(_set_pos!=0xFF && _set_stop==false){
              _no_calibarte=true;
               ESP_LOGE(TAG,"Motor not calibrated !!!");   
            }
            if(data==0){
               temp_operation = cover::COVER_OPERATION_OPENING;
               _set_open=false;
            } else if(data==1){
               temp_operation = cover::COVER_OPERATION_IDLE;
               _set_pos=0xFF; // сбросить запрос позиционирования
               _set_stop=false;
            } else if(data==2){
               temp_operation = cover::COVER_OPERATION_CLOSING;
               _set_close=false;
            } else {
               break;
            } 
            if(_no_calibarte){ // в случае отсутствия калибровки
               _set_pos=0xFF; // сбросить запрос позиционирования
            }                
            ret = true;
            break;
         }
         case idQue_Percent: { // ответ на наш запрос позиционирования
            ESP_LOGD(TAG,"Confirm GOTO POSITION:%d",data);
            if(data<=100){
              #ifdef TCURT_VIRTUAL_POS
               _timer_pos=esphome::millis(); //засекаем время начала позиионирования
               if(_speed!=0 && _old_pos!=0xFF && _no_calibarte==false){ // скорость рассчитанна, старое положение есть, двигатель калиброван
                  _dest_pos=data; // активация рассчета положения привода
               }
              #endif
               _set_pos=0xFF;
               float quePos=(float)data/100.0;
               if(quePos > this->position){
                  temp_operation = cover::COVER_OPERATION_OPENING;
               } else {
                  temp_operation = cover::COVER_OPERATION_CLOSING;
               }
            } else {
               break;
            }
            ret = true;
            break;
         }
         case idRep_Percent: { // остановка устроства в положении в %
            _set_open=false;
            _set_stop=false;
            _set_close=false;
            _set_pos=0xFF;
            #ifdef TCURT_VIRTUAL_POS 
               _dest_pos=0xFF; // признак достижения позиции
            #endif
            if(data>100){
               if(_no_calibarte==false){
                  ESP_LOGE(TAG,"Motor not calibrated(1) !!!");
                  _no_calibarte=true;
               }
               #ifdef TCURT_VIRTUAL_POS
                  _speed=0; // нужно пересчитать скорость
               #endif
           } else {
               ESP_LOGD(TAG,"Get POSITION:%d",data);
               _no_calibarte=false;
               temp_position = (float)data/100.0;
               #ifdef TCURT_VIRTUAL_POS 
               // тут рассчет скорости перемещения
                  if(_old_pos!=0xFF && _timer_pos!=0){
                     uint32_t deltatime=esphome::millis()-_timer_pos;
                     _timer_pos=0;
                     if(deltatime>0){
                        float deltapos=abs((float)_old_pos-data);
                        if(deltapos>2){
                           if(_speed==0){
                              _speed=deltapos/deltatime;
                           } else {
                               constexpr float koef=0.1;
                              _speed=_speed*(1.0-koef)+(deltapos*koef/deltatime); // ползущий фильтр
                           }
                           ESP_LOGV(TAG,"Calculated speed: %f",_speed); 
                        }
                     }
                  }
                  _old_pos=data;
               #endif
            }
            temp_operation = cover::COVER_OPERATION_IDLE;
            ret = true;
            break;
         }
         case idMode: { // утро-ночь
          #ifdef TCURT_NDMODE   
            ESP_LOGD(TAG,"Confirm MODE:%d",data);
            if(data==_set_mode){
               _set_mode=0xFF;
            }
            ret = true;
          #else
            ESP_LOGE(TAG,"Unsupported MODE:%d",data);
          #endif
            break;
         }
         case idDir: { // настрока направления вращения мотора
            ESP_LOGD(TAG,"Confirm MOTOR DIRECTION:%d",data);
            if(data!=TCURT_MOTOR_REVERS){
               sendComm(idDir, dtEnum, (uint8_t) TCURT_MOTOR_REVERS); // отправляем настройку реверса
            } else if(sendCounter==5) { // получили подтверждение правильной настройки реверса
               #ifdef TCURT_RESTORE // режим восстановления после отключения питания
                  #ifndef TCURT_SILENT_CALIBRATE // тихая калибровка не включена
                     if(restoreCounter==0){ // запуск проедуры восстановления позиции
                        restoreCounter=1;
                        ESP_LOGD(TAG,"Start calibrate");
                     }
                  #endif
               #endif
               sendCounter=6;
            }                
            ret = true;
            break;
         }
         case idPower: { // auto Power
          #ifdef TCURT_AUTOPOWER   
            ESP_LOGD(TAG,"Confirm AUTO POWER:%d",data);
            if(data==_set_autopower){
               _set_autopower=0xFF;
            }
            ret = true;
          #else  
            ESP_LOGE(TAG,"Unsupported AUTO POWER:%d",data);
          #endif
            break;
         }
         case idState: { // текущий режим работы открытие-закрытиие
            ESP_LOGD(TAG,"Get WORK STATE:%d",data);
            if(data==0){
               temp_operation = cover::COVER_OPERATION_OPENING;
            } else if(data==1){
               temp_operation = cover::COVER_OPERATION_CLOSING;
            } else {
               break;
            }                
            ret = true;
            break;
         }
         case idCountDown: { // время задержки команды 
          #ifdef TCURT_COUNTDOWN   
            ESP_LOGD(TAG,"Get COUNTDOWN TIME:%d hour",data);
            if(data==_set_countdown){
               _set_countdown=0xFF;
            }
            ret = true;
          #else             
            ESP_LOGE(TAG,"Unsupported COUNTDOWN TIME:%d hour",data);  
          #endif
            break;
         }
         case idLeftTime: { // оставшееся время
            ESP_LOGE(TAG,"Get unsupported LEFT TIME:%d sec",data);
            break;
         }
         case idTotalTime: { // всего времени
            ESP_LOGE(TAG,"Get unsupported TOTAL TIME:%d sec",data);
            break;
         }
         case idSituation: { // текущее положение штор
            ESP_LOGD(TAG,"Get CURRENT SITUATION:%d",data);
            if(data==ON){ //fully_close
               temp_position = cover::COVER_CLOSED;
               _set_close=false;
            } else if(data==OFF) { //fully_open
               temp_position = cover::COVER_OPEN;
               _set_open=false;
            } else {
               break;
            }
            temp_operation = cover::COVER_OPERATION_IDLE;
            ret = true;
            break;
         }
         case idFault: { // ошибка мотора
            ESP_LOGE(TAG,"Get state MOTOR FAUL %d",data);
            // тут можно добавить обработку ошибки мотора
            ret = true;
            break;
         }
         default:
           ESP_LOGE(TAG,"Get Unexpect reply 0x%X",dpid);
           break;
       }       
       if(ret && (temp_operation!=this->current_operation || temp_position!=this->position)){
          this->current_operation = temp_operation;
          this->position = temp_position;
          this->publish_state(); // если что то изменилось - публикуем
      }
       return ret;
    }

// РАЗБОР полученного пакета 
    void processSerialCommand() {
       #ifdef TCURT_PRINT_RAW_PROTO
          _debugPrintPacket(receivedCommand, receiveIndex+1, false, TCURT_PRINT_RAW_PROTO);
       #endif
       if (commandLength > -1) {
          bool knownCommand = false;
          proto_vers=receivedCommand[2]; // запоминаем версию протокола MCU
          if(proto_vers==0x03){
             if (receivedCommand[3] == DATAPOINT_REPORT) { // пакет данных функционального назначения
               uint16_t dataSize=get16(receivedCommand+4); //размер посылки без хидера
               uint16_t valSize=get16(receivedCommand+8); // размер буфера значения
               if(dataSize-valSize==4){ // размеры соответствуют друг другу
                  if (valSize==1 && (receivedCommand[7]==dtBool || receivedCommand[7]==dtEnum || receivedCommand[7]==dtBits)){ // проверка соответствия типу даннных
                     knownCommand = processStatusCommand(receivedCommand[6], receivedCommand[10]); // однобайтовое значение
                  } else if(valSize==4 && receivedCommand[7]==dtVal){
                     knownCommand = processStatusCommand(receivedCommand[6], get32(receivedCommand+10)); // 32битное значение
                  }
               }
             } else { // общий пакет протокола туя
               knownCommand = processCommand(receivedCommand[3], receivedCommand[5]);
             }
          } else {
             ESP_LOGE(TAG,"Unsupported proto version");           
          }
          if (!knownCommand) {
            ESP_LOGE(TAG,"Unknown command");
            _debugPrintPacket(receivedCommand, receiveIndex+1, false, ESPHOME_LOG_LEVEL_ERROR);
          }
       }
    }

// сюда пихаем байт присланый MCU
    void inData(uint8_t inChar){
      if((int16_t)(sizeof(receivedCommand))>receiveIndex){ // контроль переполнения буфера входных данных
         receiveIndex++;
      }
      receivedCommand[receiveIndex] = inChar;
      if(receiveIndex==0 && COMMAND_START[0] != inChar){ //проверка хидера 55
         resetAll(); //  не совпало - ресетимся
      } else if (receiveIndex==1) { // проверка хидера 55 AA
        if (COMMAND_START[1]!=inChar) {
           if(COMMAND_START[0]==inChar){ // если опять 55
              receiveIndex=0; // считаем, что это нулевой байт пакета
          } else {
              resetAll(); //  не совпало - ресетимся
           }
        }
      } else if (receiveIndex == 5) { // считаем размер пакета
        commandLength = receivedCommand[4] * 0x100 + inChar;
      } else if ((commandLength > -1) && (receiveIndex == (6 + commandLength))) { // получили полный пакет
        uint8_t expChecksum = 0;         //проверяем КС
        for (uint8_t i = 0; i < receiveIndex; i++) {
          expChecksum += receivedCommand[i];
        }
        if (expChecksum == receivedCommand[receiveIndex]) {
          processSerialCommand();
          if(waitReply==receivedCommand[3]){ // пришел ответ на запрос
             sendRight=true; // можно отправлять новые пакеты     
          }
        }
        resetAll();
      }
    }
    
    uint8_t getSavedPos(){ // получить сохраненную позицию с проверкой и исправлением для калибровки
       #ifdef TCURT_RESTORE
          if(saveData.pos>=0 && saveData.pos<=100){
             return saveData.pos;
          }
       #endif
       return 100;
    }

 protected:

    void control(const cover::CoverCall &call) override {
       #ifdef TCURT_RESTORE
          #ifdef TCURT_SILENT_CALIBRATE
             if(restoreCounter==0){ //в молчаливом режиме, калибровки еще не было
                if (call.get_position().has_value()) { // получили запрос нового положения
                   _set_pos = *call.get_position()*100; 
                   if (_no_calibarte==false){
                      restoreCounter=6;
                      ESP_LOGD(TAG,"Silent calibrate finished");
                   } else if(_set_pos<100 && _set_pos>0){ // если не крайние точки - запустить калибровку
                      saveDataFlash(_set_pos); // финальная точка калибровки 
                      restoreCounter=2;
                      ESP_LOGD(TAG,"Silent calibrate started");
                   } else { //если запрошена крайняя точка - разрешить без калибровки
                      ESP_LOGD(TAG,"Requested edge point in silent calibrate");
                   }
                }
                if (call.get_stop()) { //останов разрешить без калибровки
                   _set_stop=true;   
                }
                this->publish_state();
                return;
             } else 
          #endif          
             if(restoreCounter<6){
                this->publish_state();
                return;
             }
       #endif
       if (call.get_position().has_value()) { // получили запрос нового положения
          float target = *call.get_position(); 
          _set_pos=(uint8_t)(100*target);
       }
       if (call.get_stop()) {
          _set_stop=true;   
       }
       this->publish_state();
    }

 public:
   
   cover::CoverTraits get_traits() override {
     auto traits = cover::CoverTraits();
     traits.set_supports_position(true);
     traits.set_supports_stop(true);
     traits.set_is_assumed_state(true);
     return traits;
   }
  
    void dump_config() override{
        LOG_COVER("", "Tuya Curtain (proto3)", this);
        LOG_TEXT_SENSOR("  ", "MCU product ID", this->sensor_mcu_id_);
        #ifdef TCURT_RESTORE
           ESP_LOGCONFIG(TAG, "Restore state enabled, save data perid: %d uS",TCURT_RESTORE);
        #endif
        #ifdef TCURT_VIRTUAL_POS
           ESP_LOGCONFIG(TAG, "Public move position, every: %d uS",TCURT_VIRTUAL_POS);
        #endif
        if(status_pin_reported_!=-1){
           char buff[]={(char)('A'+status_pin_reported_/0x100),0}; 
           ESP_LOGCONFIG(TAG, "The MCU requires the use of a P%s_%u output to indicate WiFi status",buff,(uint8_t)status_pin_reported_);    
           if(status_pin_==nullptr){ 
              ESP_LOGE(TAG, "You need to configure WiFi status output pin");    
           }
        }
        if(reset_pin_reported_!=-1){
           char buff[]={(char)('A'+reset_pin_reported_/0x100),0}; 
           ESP_LOGCONFIG(TAG, "The MCU requires the use of a P%s_%u as reset protocol input",buff,(uint8_t)reset_pin_reported_);    
           if(status_pin_==nullptr){ 
              ESP_LOGE(TAG, "You need to configure reset input pin");    
           }
        }
    }

    void initCurtain(esphome::uart::UARTComponent *parent = nullptr) { // инициализация объекта
        _tuya_serial = parent;
    }
    float get_setup_priority() const override { return esphome::setup_priority::DATA;}
    void set_product_id_text(text_sensor::TextSensor *sensor){sensor_mcu_id_=sensor;}

    void setup() override {
       sendCounter=1; // нужно запустить карусель обмена данными
       this->current_operation = cover::COVER_OPERATION_IDLE;
       #ifdef TCURT_RESTORE
          if(loadDataFlash()){
             this->position = float(getSavedPos())/100.0;  
          } else 
       #endif       
          this->position = cover::COVER_CLOSED;  
       this->publish_state();
    }

    void loop() override {

        // читаем UART порт
        if (_tuya_serial!=nullptr){
           while(_tuya_serial->available()){ //читаем
              uint8_t data;
              _tuya_serial->read_byte(&data);
              lastRead=esphome::millis();//время чтения последнего байта
              inData(data); // обрабатываем
           }               
        }

        //if(esphome::millis()<40000){
        //   return;
        //}
        
      #ifdef TCURT_VIRTUAL_POS
        // рассчет текущего положения во время позиционирования
        if(_no_calibarte==false && _dest_pos!=0xFF){
           uint32_t _now=esphome::millis();
           if(_now-_timer_update>=TCURT_VIRTUAL_POS){
              _timer_update=_now;
              float calc_pos=_speed*(_now-_timer_pos); // пройденный путь
              if(_old_pos<_dest_pos){ // закрывается
                 calc_pos=calc_pos+_old_pos;   
              } else { // открывается
                 calc_pos=-calc_pos+_old_pos;
              }
              if(calc_pos<=100.0 && calc_pos>=0.0){ // фильтрация бреда
                 this->position = calc_pos/100.0;
                 ESP_LOGV(TAG,"New calculated position:%f",this->position);
                 this->publish_state();
              }
           }
        }
      #endif  
      #ifdef TCURT_RESTORE // режим восстановления после отключения питания
        if(restoreCounter==6 && this->current_operation == cover::COVER_OPERATION_IDLE){
            if(_no_calibarte==false && esphome::millis()-saveTimer>=TCURT_RESTORE){ //мотор калиброван
                if(_set_open==false && _set_close==false && _set_pos==0xFF && _set_stop==false){
                    saveTimer=esphome::millis(); // пытаемся сохранить данные по таймеру и только в останове
                    saveDataFlash();
                }
            }
        } else if(this->current_operation == cover::COVER_OPERATION_IDLE || esphome::millis()-waiteTimer>(uint32_t)TCURT_CALIBRATE_TIMEOT*(calibrate_count+1)){
            if(_set_open==false && _set_close==false && _set_pos==0xFF && _set_stop==false){
              waiteTimer=esphome::millis();
              if(restoreCounter==1){ // попытка установить в текущую позиию, а заодно выяснить калибровку мотора
                 ESP_LOGD(TAG,"Restore position probe...");
                 _set_pos=getSavedPos();
                 restoreCounter=2;
              } else if(restoreCounter==2){
                 if(_no_calibarte){ 
                    if(getSavedPos()!=0){
                       ESP_LOGD(TAG,"Find full open position(1)...");
                       _set_pos=0;               
                    } else {
                       ESP_LOGD(TAG,"Find full close position(1)...");
                       _set_pos=100;               
                    }
                    restoreCounter=3;   
                 } else {
                    ESP_LOGD(TAG,"Calibration finished(1)");
                    restoreCounter=6;
                 }                     
              } else if(restoreCounter==3){
                 if(_no_calibarte){ 
                    if(getSavedPos()!=0){
                       ESP_LOGD(TAG,"Find full close position(2)...");
                       _set_pos=100;               
                    } else {
                       ESP_LOGD(TAG,"Find full open position(2)...");
                       _set_pos=0;               
                    }
                 } 
                 restoreCounter=4;
              } else if(restoreCounter==4){              
                 restoreCounter=5;
              } else if(restoreCounter==5){              
                if(_no_calibarte==false){
                   if(getSavedPos()!=(uint8_t)(this->position*100)){
                      ESP_LOGD(TAG,"Restore position...");
                      _set_pos=getSavedPos();
                   } 
                   ESP_LOGD(TAG,"Calibration finished(2)");
                   restoreCounter=6;
                } else {
                   ESP_LOGE(TAG,"Calibration error. iteration: %d, repeat...",calibrate_count);
                   if(++calibrate_count<10){
                      _set_pos=50; 
                      restoreCounter=2;
                   } else {
                      if(calibrate_count>0){
                         ESP_LOGE(TAG,"Recommendation: increase the \'clibrate_timeout\' to %d uS",(uint32_t)TCURT_CALIBRATE_TIMEOT*calibrate_count);
                      }
                      restoreCounter=6;
                   }
                }
              }
           }
        }
      #endif
      
        // карусель обмена данными
        if(esphome::millis()-lastSend>UART_TIMEOUT && (esphome::millis()-lastRead>UART_TIMEOUT || sendRight)){ //можно отправлять
          if(sendCounter==1){ // отправим первый heatbear
             ESP_LOGD(TAG,"Send first HEARTBEAT");          
             sendComm(HEARTBEAT); 
          } else if(sendCounter==2){
             ESP_LOGD(TAG,"Send Prod_ID request");          
             sendComm(PRODUCT_QUERY);
          } else if(sendCounter==3){
             ESP_LOGD(TAG,"Send State request");          
             sendComm(CONF_QUERY);
          } else if(sendCounter==4){ // отправка стартового сетевого состояния
             ESP_LOGD(TAG,"Send WiFi_OK state");          
             sendNetState(getNetState());
          } else if(sendCounter==5){ // отправка настроки реверса моттора
             ESP_LOGD(TAG,"Send Motor Reverse state");          
             sendComm(idDir, dtEnum, (uint8_t) TCURT_MOTOR_REVERS); // отправляем настройку реверса
          } else if(sendCounter==6){ // остальной процессинг

            if(esphome::millis()-lastSend>SEND_TIMEOUT){
                static uint32_t lastSendPing=esphome::millis();
                if(_set_stop ){
                   ESP_LOGD(TAG,"Send STOP(2)");
                   sendComm(idControl, dtEnum, tenStop); // остановить   
                } else if (_set_pos!=0xFF) {
                   ESP_LOGD(TAG,"Send GOTO POSITION(2): %d",_set_pos);
                   sendComm(idQue_Percent, dtVal, (uint8_t)(_set_pos)); // запрос установки в позиию
                } else if (_set_open) {
                   ESP_LOGD(TAG,"Send OPEN");
                   sendComm(idControl, dtEnum, tenOpen); // открыть   
                } else if (_set_close) {
                   ESP_LOGD(TAG,"Send CLOSE");
                   sendComm(idControl, dtEnum, tenClose); // закрыть  
            #ifdef TCURT_NDMODE   
                } else if (_set_mode!=0xFF){
                   ESP_LOGD(TAG,"Send mode Morning/Night(2): %d", _set_mode);
                   sendComm(idMode, dtEnum, _set_mode); // день/ночь  
            #endif
            #ifdef TCURT_AUTOPOWER   
                } else if (_set_autopower!=0xFF){
                   ESP_LOGD(TAG,"Send Auto Power(2): %d", _set_autopower);
                   sendComm(idPower, dtBool, _set_autopower); // авто повер
            #endif
            #ifdef TCURT_COUNTDOWN   
                } else if (_set_countdown!=0xFF){
                   ESP_LOGD(TAG,"Send CountDown(2): %d", _set_countdown);
                   sendComm(idCountDown, dtEnum, _set_countdown); // таймер
            #endif
                } else if(esphome::millis()-lastSendPing>HEARTBEAT_INTERVAL*1000){
                   ESP_LOGV(TAG,"Send regular HEARTBEAT");          
                   sendComm(HEARTBEAT);
                   lastSendPing+=HEARTBEAT_INTERVAL*1000;
                }
             }
          }
        }
    }

};

}  // namespace tuya_curtian
}  // namespace esphome



#endif //TUYA_TERMO_H
