/**
 * IR Driver
 */
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <algorithm>
#include <string>


// Configuration parameters

#define REPORT_UNKNOWNS     true  // Report inbound IR messages that we don't know.
#define REPORT_RAW_UNKNOWNS true  // Report the whole buffer, recommended:

// Let's use a larger than normal buffer so we can handle AirCon remote codes.
const uint16_t kCaptureBufferSize = 1024;

// Ignore unknown messages with <10 pulses (see also REPORT_UNKNOWNS)
const uint16_t kMinUnknownSize = 2 * 10;
const uint8_t kCaptureTimeout = 16;  // Milliseconds

#define argType "type"
#define argData "code"
#define argBits "bits"
#define argRepeat "repeats"

uint64_t getUInt64fromHex(char const *str);
bool sendIRCode(int const ir_type, uint64_t const code, char const * code_str, uint16_t bits, uint16_t repeat);

IRsend irsend = IRsend(IR_SEND_PIN);
IRrecv irrecv(IR_RECEIVE_PIN, kCaptureBufferSize, kCaptureTimeout, true);
decode_results capture;  // Somewhere to store inbound IR messages.

bool    ir_lock        = false;  // Primitive locking for gating the IR LED.
String  lastIrReceived = "None";


// Debug messages get sent to the serial port.
void debug(String str) {
  Homie.getLogger() << "IR Service: " << str << endl;
}

// Count how many values are in the String.
// Args:
//   str:  String containing the values.
//   sep:  Character that separates the values.
// Returns:
//   The number of values found in the String.
uint16_t countValuesInStr(const String str, char sep) {
  int16_t index = -1;
  uint16_t count = 1;
  do {
    index = str.indexOf(sep, index + 1);
    count++;
  } while (index != -1);
  return count;
}

// Dynamically allocate an array of uint16_t's.
// Args:
//   size:  Nr. of uint16_t's need to be in the new array.
// Returns:
//   A Ptr to the new array. Restarts the ESP8266 if it fails.
uint16_t * newCodeArray(const uint16_t size) {
  uint16_t *result;

  result = reinterpret_cast<uint16_t*>(malloc(size * sizeof(uint16_t)));
  // Check we malloc'ed successfully.
  if (result == NULL) {  // malloc failed, so give up.
    Serial.printf("\nCan't allocate %d bytes. (%d bytes free)\n",
                  size * sizeof(uint16_t), ESP.getFreeHeap());
    Serial.println("Giving up & forcing a reboot.");
    
    Homie.reset();  // Reboot.

    delay(500);  // Wait for the restart to happen.
    return result;  // Should never get here, but just in case.
  }
  return result;
}

// Parse a GlobalCache String/code and send it.
// Args:
//   str: A GlobalCache formatted String of comma separated numbers.
//        e.g. "38000,1,1,170,170,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,
//              20,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,20,20,20,20,63,
//              20,20,20,20,20,20,20,20,20,20,20,20,20,63,20,20,20,63,20,63,20,
//              63,20,63,20,63,20,63,20,1798"
//        Note: The leading "1:1,1," of normal GC codes should be removed.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendGC(const String str) {
  uint16_t count;
  uint16_t *code_array;
  String tmp_str;

  // Remove the leading "1:1,1," if present.
  if (str.startsWith("1:1,1,"))
    tmp_str = str.substring(6);
  else
    tmp_str = str;

  // Find out how many items there are in the string.
  count = countValuesInStr(tmp_str, ',');

  // Now we know how many there are, allocate the memory to store them all.
  code_array = newCodeArray(count);

  // Now convert the strings to integers and place them in code_array.
  count = 0;
  uint16_t start_from = 0;
  int16_t index = -1;
  do {
    index = tmp_str.indexOf(',', start_from);
    code_array[count] = tmp_str.substring(start_from, index).toInt();
    start_from = index + 1;
    count++;
  } while (index != -1);

  irsend.sendGC(code_array, count);  // All done. Send it.
  free(code_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}

// Parse a Pronto Hex String/code and send it.
// Args:
//   str: A comma-separated String of nr. of repeats, then hexadecimal numbers.
//        e.g. "R1,0000,0067,0000,0015,0060,0018,0018,0018,0030,0018,0030,0018,
//              0030,0018,0018,0018,0030,0018,0018,0018,0018,0018,0030,0018,
//              0018,0018,0030,0018,0030,0018,0030,0018,0018,0018,0018,0018,
//              0030,0018,0018,0018,0018,0018,0030,0018,0018,03f6"
//              or
//              "0000,0067,0000,0015,0060,0018". i.e. without the Repeat value
//        Requires at least kProntoMinLength comma-separated values.
//        sendPronto() only supports raw pronto code types, thus so does this.
//   repeats:  Nr. of times the message is to be repeated.
//             This value is ignored if an embeddd repeat is found in str.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendPronto(const String str, uint16_t repeats) {
  uint16_t count;
  uint16_t *code_array;
  int16_t index = -1;
  uint16_t start_from = 0;

  // Find out how many items there are in the string.
  count = countValuesInStr(str, ',');

  // Check if we have the optional embedded repeats value in the code string.
  if (str.startsWith("R") || str.startsWith("r")) {
    // Grab the first value from the string, as it is the nr. of repeats.
    index = str.indexOf(',', start_from);
    repeats = str.substring(start_from + 1, index).toInt();  // Skip the 'R'.
    start_from = index + 1;
    count--;  // We don't count the repeats value as part of the code array.
  }

  // We need at least kProntoMinLength values for the code part.
  if (count < kProntoMinLength) return false;

  // Now we know how many there are, allocate the memory to store them all.
  code_array = newCodeArray(count);

  // Rest of the string are values for the code array.
  // Now convert the hex strings to integers and place them in code_array.
  count = 0;
  do {
    index = str.indexOf(',', start_from);
    // Convert the hexadecimal value string to an unsigned integer.
    code_array[count] = strtoul(str.substring(start_from, index).c_str(),
                                NULL, 16);
    start_from = index + 1;
    count++;
  } while (index != -1);

  irsend.sendPronto(code_array, count, repeats);  // All done. Send it.
  free(code_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}


// Parse an IRremote Raw Hex String/code and send it.
// Args:
//   str: A comma-separated String containing the freq and raw IR data.
//        e.g. "38000,9000,4500,600,1450,600,900,650,1500,..."
//        Requires at least two comma-separated values.
//        First value is the transmission frequency in Hz or kHz.
// Returns:
//   bool: Successfully sent or not.
bool parseStringAndSendRaw(const String str) {
  uint16_t count;
  uint16_t freq = 38000;  // Default to 38kHz.
  uint16_t *raw_array;

  // Find out how many items there are in the string.
  count = countValuesInStr(str, ',');

  // We expect the frequency as the first comma separated value, so we need at
  // least two values. If not, bail out.
  if (count < 2)  return false;
  count--;  // We don't count the frequency value as part of the raw array.

  // Now we know how many there are, allocate the memory to store them all.
  raw_array = newCodeArray(count);

  // Grab the first value from the string, as it is the frequency.
  int16_t index = str.indexOf(',', 0);
  freq = str.substring(0, index).toInt();
  uint16_t start_from = index + 1;
  // Rest of the string are values for the raw array.
  // Now convert the strings to integers and place them in raw_array.
  count = 0;
  do {
    index = str.indexOf(',', start_from);
    raw_array[count] = str.substring(start_from, index).toInt();
    start_from = index + 1;
    count++;
  } while (index != -1);

  irsend.sendRaw(raw_array, count, freq);  // All done. Send it.
  free(raw_array);  // Free up the memory allocated.
  if (count > 0)
    return true;  // We sent something.
  return false;  // We probably didn't.
}


void irDriverSetup(void) {
  irsend.begin();
  irsend.calibrate();
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);

  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.enableIRIn();  // Start the receiver

  pinMode(IR_SEND_PIN, OUTPUT); //IR TX pin as output
  digitalWrite(IR_SEND_PIN, LOW); //turn off IR output initially

}

// Listen and decode detected IR messages
void irDriverLoop(void) {

  // Check if an IR code has been received via the IR RX module.
  if (irrecv.decode(&capture)) {
    uint32_t usecs = 0;
    
    gsReceiverString = String(capture.decode_type) + "," + resultToHexidecimal(&capture);
    if (capture.decode_type == UNKNOWN) {
      gsReceiverString += ";";
      for (uint16_t i = 1; i < capture.rawlen; i++) {        
        usecs = 0;
        for (usecs = capture.rawbuf[i] * kRawTick; usecs > UINT16_MAX; usecs -= UINT16_MAX) {
          gsReceiverString += uint64ToString(UINT16_MAX);
          gsReceiverString += ",0,";
        }
        gsReceiverString += uint64ToString(usecs, 10);
        if (i < capture.rawlen - 1) {
          gsReceiverString += ",";
        }
      }
    }

    // If it isn't an AC code, add the bits.
    if (!hasACState(capture.decode_type)) {
      gsReceiverString += "," + String(capture.bits);
    }
    
    irServiceNode.setProperty("received").send(gsReceiverString);
    debug("Listener Received: " + gsReceiverString);
  }
}

// Arduino framework doesn't support strtoull(), so make our own one.
uint64_t getUInt64fromHex(char const *str) {
  uint64_t result = 0;
  uint16_t offset = 0;
  // Skip any leading '0x' or '0X' prefix.
  if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
    offset = 2;
  for (; isxdigit((unsigned char)str[offset]); offset++) {
    char c = str[offset];
    result *= 16;
    if (isdigit(c)) /* '0' .. '9' */
      result += c - '0';
    else if (isupper(c)) /* 'A' .. 'F' */
      result += c - 'A' + 10;
    else /* 'a' .. 'f'*/
      result += c - 'a' + 10;
  }
  return result;
}


// Transmit the given IR message.
//
// Args:
//   ir_type:  enum of the protocol to be sent.
//   code:     Numeric payload of the IR message. Most protocols use this.
//   code_str: The unparsed code to be sent. Used by complex protocol encodings.
//   bits:     Nr. of bits in the protocol. 0 means use the protocol's default.
//   repeat:   Nr. of times the message is to be repeated. (Not all protcols.)
// Returns:
//   bool: Successfully sent or not.
bool sendIRCode(int const ir_type, uint64_t const code, char const * code_str,
                uint16_t bits, uint16_t repeat) {

  // Create a pseudo-lock so we don't try to send two codes at the same time.
  while (ir_lock)
    delay(20);
  ir_lock = true;

  bool success = true;  // Assume success.

  // send the IR message.
  switch (ir_type) {
    case RC5:  // 1
      if (bits == 0)
        bits = kRC5Bits;
      irsend.sendRC5(code, bits, repeat);
      break;
    case RC6:  // 2
      if (bits == 0)
        bits = kRC6Mode0Bits;
      irsend.sendRC6(code, bits, repeat);
      break;
    case NEC:  // 3
      if (bits == 0)
        bits = kNECBits;
      irsend.sendNEC(code, bits, repeat);
      break;
    case SONY:  // 4
      if (bits == 0)
        bits = kSony12Bits;
      repeat = std::max(repeat, kSonyMinRepeat);
      irsend.sendSony(code, bits, repeat);
      break;
    case PANASONIC:  // 5
      if (bits == 0)
        bits = kPanasonicBits;
      irsend.sendPanasonic64(code, bits, repeat);
      break;
    case JVC:  // 6
      if (bits == 0)
        bits = kJvcBits;
      irsend.sendJVC(code, bits, repeat);
      break;
    case SAMSUNG:  // 7
      if (bits == 0)
        bits = kSamsungBits;
      irsend.sendSAMSUNG(code, bits, repeat);
      break;
    case WHYNTER:  // 8
      if (bits == 0)
        bits = kWhynterBits;
      irsend.sendWhynter(code, bits, repeat);
      break;
    case AIWA_RC_T501:  // 9
      if (bits == 0)
        bits = kAiwaRcT501Bits;
      repeat = std::max(repeat, kAiwaRcT501MinRepeats);
      irsend.sendAiwaRCT501(code, bits, repeat);
      break;
    case LG:  // 10
      if (bits == 0)
        bits = kLgBits;
      irsend.sendLG(code, bits, repeat);
      break;
    case MITSUBISHI:  // 12
      if (bits == 0)
        bits = kMitsubishiBits;
      repeat = std::max(repeat, kMitsubishiMinRepeat);
      irsend.sendMitsubishi(code, bits, repeat);
      break;
    case DISH:  // 13
      if (bits == 0)
        bits = kDishBits;
      repeat = std::max(repeat, kDishMinRepeat);
      irsend.sendDISH(code, bits, repeat);
      break;
    case SHARP:  // 14
      if (bits == 0)
        bits = kSharpBits;
      irsend.sendSharpRaw(code, bits, repeat);
      break;
    case COOLIX:  // 15
      if (bits == 0)
        bits = kCoolixBits;
      irsend.sendCOOLIX(code, bits, repeat);
      break;
    case DENON:  // 17
      if (bits == 0)
        bits = DENON_BITS;
      irsend.sendDenon(code, bits, repeat);
      break;
    case SHERWOOD:  // 19
      if (bits == 0)
        bits = kSherwoodBits;
      repeat = std::max(repeat, kSherwoodMinRepeat);
      irsend.sendSherwood(code, bits, repeat);
      break;
    case RCMM:  // 21
      if (bits == 0)
        bits = kRCMMBits;
      irsend.sendRCMM(code, bits, repeat);
      break;
    case SANYO_LC7461:  // 22
      if (bits == 0)
        bits = kSanyoLC7461Bits;
      irsend.sendSanyoLC7461(code, bits, repeat);
      break;
    case RC5X:  // 23
      if (bits == 0)
        bits = kRC5XBits;
      irsend.sendRC5(code, bits, repeat);
      break;
    case PRONTO:  // 25
      success = parseStringAndSendPronto(code_str, repeat);
      break;
    case NIKAI:  // 29
      if (bits == 0)
        bits = kNikaiBits;
      irsend.sendNikai(code, bits, repeat);
      break;
    case RAW:  // 30
      success = parseStringAndSendRaw(code_str);
      break;
    case GLOBALCACHE:  // 31
      success = parseStringAndSendGC(code_str);
      break;
    case MIDEA:  // 34
      if (bits == 0)
        bits = kMideaBits;
      irsend.sendMidea(code, bits, repeat);
      break;
    case MAGIQUEST:  // 35
      if (bits == 0)
        bits = kMagiquestBits;
      irsend.sendMagiQuest(code, bits, repeat);
      break;
    case LASERTAG:  // 36
      if (bits == 0)
        bits = kLasertagBits;
      irsend.sendLasertag(code, bits, repeat);
      break;
    case MITSUBISHI2:  // 39
      if (bits == 0)
        bits = kMitsubishiBits;
      repeat = std::max(repeat, kMitsubishiMinRepeat);
      irsend.sendMitsubishi2(code, bits, repeat);
      break;
    case GICABLE:  // 43
      if (bits == 0)
        bits = kGicableBits;
      repeat = std::max(repeat, kGicableMinRepeat);
      irsend.sendGICable(code, bits, repeat);
      break;
    case LUTRON:  // 47
      if (bits == 0)
        bits = kLutronBits;
      irsend.sendLutron(code, bits, repeat);
      break;
    case PIONEER:  // 50
      if (bits == 0)
        bits = kPioneerBits;
      irsend.sendPioneer(code, bits, repeat);
      break;
    case LG2:  // 51
      if (bits == 0)
        bits = kLgBits;
      irsend.sendLG2(code, bits, repeat);
      break;
    default:
      // If we got here, we didn't know how to send it.
      success = false;
  }

  // Release the lock.
  ir_lock = false;

  // Indicate that we sent the message or not.
  if (success) {
    debug("Sent the IR message:");
  } else {
    debug("Failed to send IR Message:");
  }

  debug("Type: " + String(ir_type));
  // For "long" codes we basically repeat what we got.
  if (hasACState((decode_type_t) ir_type) ||
      ir_type == PRONTO ||
      ir_type == RAW ||
      ir_type == GLOBALCACHE) {
    debug("Code: ");
    debug(code_str);
    // Confirm what we were asked to send was sent.
    if (success) {
      if (ir_type == PRONTO && repeat > 0) {
        gsCommandString = String(ir_type) + ",R" + String(repeat) + "," + String(code_str);
      } else {
        gsCommandString = String(ir_type) + "," + String(code_str);
      }
    }
  } else {  // For "short" codes, we break it down a bit more before we report.
    debug("Code: 0x" + uint64ToString(code, 16));
    debug("Bits: " + String(bits));
    debug("Repeats: " + String(repeat));
    if (success) {
      gsCommandString = String(ir_type) + "," + uint64ToString(code, 16) + "," + String(bits) + "," +  String(repeat);
    }
  }
  return success;
}


/**
 * Decode and send the received IR Codes
 */
bool processCommand(String const commandString) {
  char* tok_ptr;
  uint64_t code = 0;
  uint16_t nbits = 0;
  uint16_t repeat = 0;

  // Make a copy of the callback string as strtok destroys it.
  char* callback_c_str = strdup(commandString.c_str());

  // Get the numeric protocol type.
  int ir_type = strtoul(strtok_r(callback_c_str, ",", &tok_ptr), NULL, 10);
  char* next = strtok_r(NULL, ",", &tok_ptr);

  // If there is unparsed string left, try to convert it assuming it's hex.
  if (next != NULL) {
    code = getUInt64fromHex(next);
    next = strtok_r(NULL, ",", &tok_ptr);
  } else {
    // We require at least two value in the string. Give up.
    return false;
  }
  
  // If there is still string left, assume it is the bit size.
  if (next != NULL) {
    nbits = atoi(next);
    next = strtok_r(NULL, ",", &tok_ptr);
  }
  
  // If there is still string left, assume it is the repeat count.
  if (next != NULL)
    repeat = atoi(next);

  free(callback_c_str);

  // send received value by IR signal
  return sendIRCode(ir_type, code, commandString.substring(commandString.indexOf(",") + 1).c_str(), nbits, repeat);
}
