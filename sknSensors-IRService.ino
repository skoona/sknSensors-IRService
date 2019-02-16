/**
 * Module: sknSensors-IRService.ino
 *  Data:  data/homie/config.json, 
 *         data/homie/ui_bundle.gz
 *  Date:  02/16/2019
 *  
 *  Description: 
 *  - Broadcast IR Commands as received over command property and echo processed command-string back on received property
 *  
 *  On 'command/set' 
 *    process IR command <string-command>
 * 
 * On `IO Listener'
 *    set 'received' <string-command>
 *  
 * On `IO Decdoer'
 *    set 'decoded' <string-command>
 *  
 * Customization:
 *  1. Change/Update NODE_* defines to indicate the intended room placement            -- OPTIONAL
 *  2. Change/Update device_id: in config.json to indicate the intended room placement -- SUGGESTED
 *  3. Ensure senors are are wired properly or change DHT_PIN and RCWL_PIN             -- REQUIRED
 */
#include <Homie.h>

#ifdef ESP8266
extern "C" {
#include "user_interface.h" // to set CPU Freq
}
#endif

#define NODE_SERVICE_NAME   "IR Broadcaster"
#define NODE_LISTENER_NAME  "IR Listener"
#define NODE_DECODER_NAME   "IR Decoder"

#define NODE_SENSOR_INTERVAL_MS     250
#define NODE_SENSOR_INTERVAL_MIN_MS 99
#define NODE_SENSOR_INTERVAL_MAX_MS 1000

#define FW_NAME "sknSensors-IRService"
#define FW_VERSION  "0.0.5"

#define IR_SEND_PIN    D6      // GPIO12
#define IR_RCV_PIN     D6      // GPIO12
#define IR_DECODER_PIN D6      // GPIO12
 
String  gsCommandString   = "";
String  gsReceiverString  = "";
String  gsDecoderString   = "";

// HomieNode(const char* id, const char* name, const char* type, bool range = false, uint16_t lower = 0, uint16_t upper = 0, const HomieInternals::NodeInputHandler& nodeInputHandler = [](const HomieRange& range, const String& property, const String& value) { return false; });
HomieNode irServiceNode("irservice",    NODE_SERVICE_NAME,  "theater-remote");

HomieSetting<long> sensorsIntervalSetting("sensorsInterval", "The interval in seconds to wait between sending commands.");

bool broadcastHandler(const String& level, const String& value) {
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}


bool commandHandler(const HomieRange& range, const String& value) {

  delay( sensorsIntervalSetting.get() );

  Homie.getLogger() << "commandHandler() sending: " << value << endl;

  return true;
}

void setupHandler() {    
  yield();  
}

void loopHandler() {
// irServiceNode
// irServiceNode
}


void setup() {
  delay(50);
  system_update_cpu_freq(SYS_CPU_160MHZ);
  yield();
  
  Serial.begin(115200);
  Serial << endl << endl;
  yield();
  
  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie_setBrand("sknSensors");
  
  Homie.setSetupFunction(setupHandler)
       .setLoopFunction(loopHandler)
       .setBroadcastHandler(broadcastHandler);
                                    
  sensorsIntervalSetting.setDefaultValue(NODE_SENSOR_INTERVAL_MS)       // more than zero & less than one hour
                        .setValidator([] (long lValue) { 
                          return (lValue > NODE_SENSOR_INTERVAL_MIN_MS) && 
                                 (lValue <= NODE_SENSOR_INTERVAL_MAX_MS);
                        });

  irServiceNode.advertise("command").setName(NODE_SERVICE_NAME)
                              .settable(commandHandler)
                              .setDatatype("string")
                              .setRetained(false)
                              .setUnit("%s");

  irServiceNode.advertise("received").setName(NODE_LISTENER_NAME)
                                     .setDatatype("string")
                                     .setRetained(true)
                                     .setUnit("%s");

  irServiceNode.advertise("decoded").setName(NODE_DECODER_NAME)
                                  .setDatatype("string")
                                  .setRetained(true)
                                  .setUnit("%s");  
  Homie.setup();
}

void loop() {
   // put your main code here, to run repeatedly:
   Homie.loop();
}
